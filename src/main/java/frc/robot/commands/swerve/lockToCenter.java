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
    
    // Reuse these objects instead of creating new ones
    private final Rotation2d targetAngle = new Rotation2d();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
    
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
        
        // Reuse the targetAngle object by updating its value
        double angleRadians = Math.atan2(dy, dx) + Math.PI;
        // Note: Rotation2d is immutable, so we need to create a new one
        // But we can minimize allocations in other ways
        Rotation2d calculatedAngle = Rotation2d.fromRadians(angleRadians);
        
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
        double speed = Math.hypot(vx, vy);

        if(speed > MaxSpeed){
            double scale = MaxSpeed / speed;
            vx *= scale;
            vy *= scale;
        }

        SmartDashboard.putNumber("Target angle degrees", calculatedAngle.getDegrees());
        SmartDashboard.putNumber("Angle error degrees", calculatedAngle.minus(currentPose.getRotation()).getDegrees());
        
        drivetrain.setControl(faceCenter
            .withTargetDirection(calculatedAngle)
            .withVelocityX(vx)
            .withVelocityY(vy)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(idleRequest);  // Reuse the idle request
    }
}