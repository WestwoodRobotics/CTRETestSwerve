package frc.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class lockToCenter extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle faceCenter;
    public lockToCenter(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle faceCenter){
        this.drivetrain = drivetrain;
        this.faceCenter = faceCenter;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        Pose2d currentPose = drivetrain.getState().Pose;
        double dx = TrajectoryConstants.kCenterField.getX() - currentPose.getX();
        double dy = TrajectoryConstants.kCenterField.getY() - currentPose.getY();
        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx) + Math.PI);

        SmartDashboard.putNumber("Target angle degrees", targetAngle.getDegrees());
        SmartDashboard.putNumber("Angle error degrees", targetAngle.minus(currentPose.getRotation()).getDegrees());
        
        drivetrain.setControl(faceCenter.withTargetDirection(targetAngle));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
