package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import pabeles.concurrency.IntOperatorTask.Max;

public class lockToCenter extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle faceCenter;
    private final DoubleSupplier orbitDoubleSupplier;
    private final DoubleSupplier radiusDoubleSupplier;
    private double MaxAngularRate;
    private double MaxSpeed;
    public lockToCenter(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle faceCenter, DoubleSupplier orbitDoubleSupplier, DoubleSupplier radiusDoubleSupplier, double maxAngleRate, double maxSpeed){
        this.drivetrain = drivetrain;
        this.faceCenter = faceCenter;
        this.orbitDoubleSupplier = orbitDoubleSupplier;
        this.radiusDoubleSupplier = radiusDoubleSupplier;
        this.MaxAngularRate = maxAngleRate;
        this.MaxSpeed = maxSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        Pose2d currentPose = drivetrain.getState().Pose;
        double dx = TrajectoryConstants.kCenterField.getX() - currentPose.getX();
        double dy = TrajectoryConstants.kCenterField.getY() - currentPose.getY();
        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx) + Math.PI);
        double radius = Math.hypot(dx, dy);
        
        double input = orbitDoubleSupplier.getAsDouble();
        double radiusinput = radiusDoubleSupplier.getAsDouble();

        double tangentUx = -dy / radius;
        double tangentUy = dx / radius; 

        double radiusUx = dx / radius;
        double radiusUy = dy / radius;
        
        double angularRate = MaxAngularRate * input;
        double tangentialSpeed = angularRate * radius;
        

        double radiusSpeed = radiusinput * MaxSpeed;
       
        double vx = tangentUx*tangentialSpeed + radiusUx*radiusSpeed;
        double vy = tangentUy*tangentialSpeed + radiusUy*radiusSpeed;
        if(vx > MaxSpeed){
            vx = MaxSpeed;
        }
        if(vy > MaxSpeed){
            vy = MaxSpeed;
        }

        SmartDashboard.putNumber("Target angle degrees", targetAngle.getDegrees());
        SmartDashboard.putNumber("Angle error degrees", targetAngle.minus(currentPose.getRotation()).getDegrees());
        
        drivetrain.setControl(faceCenter
        .withTargetDirection(targetAngle)
        .withVelocityX(vx)
        .withVelocityY(vy)
        );


    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
