package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class followObject extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric followObj;
    private final DoubleSupplier xDoubleSupplier;
    private final DoubleSupplier YDoubleSupplier;
    private double MaxAngularRate;
    private double MaxSpeed;

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
        double dx = currentPose.getX() - TrajectoryConstants.kCenterField.getX();
        double dY = currentPose.getY() - TrajectoryConstants.kCenterField.getY();
        
        double distance = Math.hypot(dx, dY);
        
        double vx = xInput * MaxSpeed;
        double vy = yInput * MaxSpeed;

        drivetrain.setControl(followObj
        .withVelocityX(vx)
        .withVelocityY(vy)
        );
    }
}
