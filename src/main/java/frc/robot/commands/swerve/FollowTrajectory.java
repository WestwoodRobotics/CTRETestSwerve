package frc.robot.commands.swerve;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Current;
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

        controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0,
              new TrapezoidProfile.Constraints(1, 0.5)));

        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        startingPose = new Pose2d(
            drivetrain.getState().Pose.getX(),
            drivetrain.getState().Pose.getY(),
            drivetrain.getState().Pose.getRotation());

        endingPose = startingPose.transformBy(
            new Transform2d(
                new Translation2d(1.0, 1.0), // Move forward 3 meters
                new Rotation2d(0.0) // No change in orientation
            )
        );

        trajectory = createTrajectory();
    }

    @Override
    public void initialize(){
        timer.start();
    }

    private Trajectory createTrajectory() {

        double intialSpeedX = drivetrain.getState().Speeds.vxMetersPerSecond;
        double intialSpeedY = drivetrain.getState().Speeds.vyMetersPerSecond;
        double iSpeed = Math.sqrt(intialSpeedY * intialSpeedY + intialSpeedX * intialSpeedX);

        TrajectoryConfig config = new TrajectoryConfig(
            MetersPerSecond.of(3.0),
            MetersPerSecondPerSecond.of(3.0));
        config.setStartVelocity(MetersPerSecond.of(iSpeed));

        List<Pose2d> waypoints = List.of(startingPose, endingPose);

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
        return timer.get() > trajectory.getTotalTimeSeconds() + 1;
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds()
                .withSpeeds(new ChassisSpeeds(0, 0, 0))
        );
    }
}
