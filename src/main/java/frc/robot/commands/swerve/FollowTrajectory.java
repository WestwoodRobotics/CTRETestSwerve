package frc.robot.commands.swerve;

import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; 

public class FollowTrajectory extends Command{

    private CommandSwerveDrivetrain drivetrain;
    private Trajectory trajectory;
    private Pose2d startingPose;
    private Pose2d endingPose; 
    private Timer timer;
    private HolonomicDriveController controller;

    public FollowTrajectory(CommandSwerveDrivetrain drivetrain){

        timer = new Timer();

        controller = TrajectoryConstants.kController;

        startingPose = new Pose2d();
        endingPose = new Pose2d();
        trajectory = new Trajectory();
        
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){

        startingPose = new Pose2d(
            drivetrain.getState().Pose.getX(),
            drivetrain.getState().Pose.getY(),
            drivetrain.getState().Pose.getRotation());

        endingPose = TrajectoryConstants.kCenterField;

        trajectory = createTrajectory(startingPose, endingPose);

        timer.restart();
    }

    private Trajectory createTrajectory(Pose2d start, Pose2d end){

        TrajectoryConfig config = new TrajectoryConfig(
            TrajectoryConstants.kMaxVelocity,
            TrajectoryConstants.kMaxAcceleration);

        List<Pose2d> waypoints = List.of(start, end);

        return TrajectoryGenerator.generateTrajectory(waypoints, config);
    }

    @Override
    public void execute(){
        Pose2d currentPose = drivetrain.getState().Pose;

        double currentTime = timer.get();
        Trajectory.State desiredState = trajectory.sample(currentTime);


        ChassisSpeeds outputs = controller.calculate(currentPose, desiredState, currentPose.getRotation());

        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(outputs)
        );
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds() + 3;
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(0, 0, 0))
        );
    }
}
