// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.swerve.Orchestrate;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.util.Color;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public Orchestra orchestra = new Orchestra();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public CANdle candle = new CANdle(50, "CANivore");

    private final SendableChooser<Command> autoChooser;
    
    public Orchestrate music = new Orchestrate(drivetrain, orchestra, "/home/lvuser/deploy/clashRoyale.chrp");

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        drivetrain.resetRotation(new Rotation2d(0));

        
        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 1.0;
        cfg.LED.StripType = StripTypeValue.GRB;

        candle.getConfigurator().apply(cfg);
        
        for (int i = 0; i < 8; i++){
            candle.setControl(new EmptyAnimation(i));
        }

        configureBindings();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                    //distance formula
                    double magnitude = Math.sqrt(
                        Math.pow(joystick.getLeftX(), 2) 
                        + Math.pow(joystick.getLeftY(), 2)
                    );

                    double angle = Math.atan2(joystick.getLeftY(), joystick.getLeftX()); // angle of joystick
                    double xMagnitude = Math.pow(magnitude,2) * Math.cos(angle); // squares magnitude, then multiplies by cos(angle) to get x mag
                    double yMagnitude = Math.pow(magnitude,2) * Math.sin(angle); // squares magnitude, then multiplies by sin(angle) to get y mag

                    return drive.withVelocityX(-(yMagnitude) * MaxSpeed) // Drive forward with squared Y (maintaining sign)
                    .withVelocityY(-(xMagnitude) * MaxSpeed) // Drive left with squared X (maintaining sign)
                    .withRotationalRate(-Math.copySign(joystick.getRightX() * joystick.getRightX(), joystick.getRightX()) * MaxAngularRate); // Drive counterclockwise with squared X (maintaining sign)
                }  
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Zero drivetrain heading on left joystick press
        joystick.rightStick().onFalse(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0))));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.rightBumper().onTrue(new InstantCommand(() -> candle.setControl(new SolidColor(0,26).withColor(new RGBWColor(Color.kBlue).scaleBrightness(1)))))
        .onFalse(new InstantCommand (() -> candle.setControl(new SolidColor(0, 26).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)))));
         joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
 
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
