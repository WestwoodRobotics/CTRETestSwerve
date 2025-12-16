package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import pabeles.concurrency.IntOperatorTask.Max;

public class lockToCenter extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle faceCenter;
    private CommandXboxController joystick;
    private double MaxSpeed;
    public lockToCenter(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle faceCenter, CommandXboxController joystick, double maxSpeed){
        this.drivetrain = drivetrain;
        this.faceCenter = faceCenter;
        this.joystick = joystick;
        this.MaxSpeed = maxSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        Pose2d currentPose = drivetrain.getState().Pose;
        double dx = TrajectoryConstants.kCenterField.getX() - currentPose.getX();
        double dy = TrajectoryConstants.kCenterField.getY() - currentPose.getY();
        Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx));
        
        double magnitude = Math.sqrt(
                        Math.pow(joystick.getLeftX(), 2) 
                        + Math.pow(joystick.getLeftY(), 2)
                    );

        double angle = Math.atan2(joystick.getLeftY(), joystick.getLeftX()); // angle of joystick
        double xMagnitude = Math.pow(magnitude,2) * Math.cos(angle); // squares magnitude, then multiplies by cos(angle) to get x mag
        double yMagnitude = Math.pow(magnitude,2) * Math.sin(angle); // squares magnitude, then multiplies by sin(angle) to get y mag
                    
        
        drivetrain.setControl(faceCenter
        .withTargetDirection(targetAngle)
        .withVelocityX(-(yMagnitude) * MaxSpeed)
        .withVelocityY(-(xMagnitude) * MaxSpeed)
        );


    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
