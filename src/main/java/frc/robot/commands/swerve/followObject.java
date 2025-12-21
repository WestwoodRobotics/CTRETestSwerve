package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class followObject extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric followObj;
    private final DoubleSupplier xDoubleSupplier;
    private final DoubleSupplier YDoubleSupplier;
    private double MaxSpeed;
    private double kp = 0.1;

    public followObject(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric followObj, DoubleSupplier xDoubleSupplier, DoubleSupplier YDoubleSupplier, double maxSpeed){
        this.drivetrain = drivetrain;
        this.followObj = followObj;
        this.xDoubleSupplier = xDoubleSupplier;
        this.YDoubleSupplier = YDoubleSupplier;
        this.MaxSpeed = maxSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        double xInput = xDoubleSupplier.getAsDouble();
        double yInput = YDoubleSupplier.getAsDouble();

        Pose2d currentPose = drivetrain.getState().Pose;
        double dx = TrajectoryConstants.kCenterField.getX() - currentPose.getX() ;
        double dy = TrajectoryConstants.kCenterField.getY() - currentPose.getY() ;
        
        double distance = Math.hypot(dx, dy);
        double heading = drivetrain.getState().Pose.getRotation().getRadians();
        double angle = Math.atan2(dy, dx);
        double angleDiff = Math.atan2(Math.sin(angle - heading), Math.cos(angle - heading));

        double vx = xInput * MaxSpeed;
        double vy = yInput * MaxSpeed;

        double joystickMag = Math.hypot(xInput, yInput);

        SmartDashboard.putNumber("angle diff", Math.abs(Math.toDegrees(angleDiff)));
        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);

        if(Math.abs(Math.toDegrees(angleDiff)) <= 75 && distance >= 2) {
            SmartDashboard.putBoolean("inview", true);
            double directionX = dx/distance;
            double directionY = dy/distance;

            SmartDashboard.putNumber("Distance", distance);
            double proportionalPullX = directionX * distance * kp * joystickMag;
            double proportionalPullY = directionY * distance * kp  * joystickMag;

            double resultX = vx + proportionalPullX;
            double resultY = vy + proportionalPullY;

            double speed = Math.hypot(resultX, resultY);
            if(speed > MaxSpeed){
                double scale = MaxSpeed / speed;
                resultX *= scale;
                resultY *= scale;
            }

            SmartDashboard.putNumber("Direction X", directionX);
            SmartDashboard.putNumber("Direction Y", directionY);
            SmartDashboard.putNumber("Resultant X", resultX);
            SmartDashboard.putNumber("Resultant Y", resultY);
            drivetrain.setControl(followObj
            .withVelocityX(resultX)
            .withVelocityY(resultY)
        );

        }
        else{
            SmartDashboard.putBoolean("inview", false);
            drivetrain.setControl(followObj
            .withVelocityX(0)
            .withVelocityY(0));
        }
        

       
    }
}
