package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
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

        double vx = xInput * MaxSpeed;
        double vy = yInput * MaxSpeed;

        drivetrain.setControl(followObj
        .withVelocityX(vx)
        .withVelocityY(vy)
        );
    }
}
