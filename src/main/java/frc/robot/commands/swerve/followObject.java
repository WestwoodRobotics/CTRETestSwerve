package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class followObject extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric followObj;
    private CommandXboxController joystick;
    private double maxAngularRate;
    private double MaxSpeed;
    private double kp = 3;

    public followObject(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric followObj, CommandXboxController joystick,double maxSpeed, double maxAngularRate){
        this.drivetrain = drivetrain;
        this.followObj = followObj;
        this.joystick = joystick;
        this.MaxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        double xInput = -joystick.getLeftX();
        double yInput = joystick.getLeftY();
        double rightX = joystick.getRightX();

        Pose2d currentPose = drivetrain.getState().Pose;
        double dx = TrajectoryConstants.kCenterField.getX() - currentPose.getX() ;
        double dy = TrajectoryConstants.kCenterField.getY() - currentPose.getY() ;
        
        double distance = Math.hypot(dx, dy);
        double heading = drivetrain.getState().Pose.getRotation().getRadians();
        double angle = Math.atan2(dy, dx);
        double angleDiff = Math.atan2(Math.sin(angle - heading), Math.cos(angle - heading));

        double vx = -xInput * MaxSpeed;
        double vy = -yInput * MaxSpeed;

        double joystickMag = Math.hypot(xInput, yInput);

        double joystickAngle = Math.toDegrees(Math.atan2(yInput,xInput)) + 90;
        joystickAngle = normalizeAngle(joystickAngle);
        SmartDashboard.putNumber("joystickangle", joystickAngle);
        SmartDashboard.putNumber("anlge", Math.toDegrees(angle));
        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);
        SmartDashboard.putNumber("diff", Math.abs(Math.toDegrees(angle) - joystickAngle));

        if(Math.abs(Math.toDegrees(angleDiff)) <= 75 && distance >= 0.5
            && Math.abs(Math.toDegrees(angle) - joystickAngle) < 90) {
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

            SmartDashboard.putNumber("Resultant X", resultX);
            SmartDashboard.putNumber("Resultant Y", resultY);
            drivetrain.setControl(followObj
            .withVelocityX(resultX)
            .withVelocityY(resultY)
            .withRotationalRate(-Math.copySign(rightX * rightX, rightX) * maxAngularRate) // Drive counterclockwise with squared X (maintaining sign))
        );

        }
        else{
            SmartDashboard.putBoolean("inview", false);
            drivetrain.setControl(followObj
            .withVelocityX(0)
            .withVelocityY(0));
        }
        

       
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
