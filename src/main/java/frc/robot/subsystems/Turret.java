package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor;
    private final TalonFX flywheelMotor;

    private final MotionMagicVoltage turretPositionRequest;
    private final MotionMagicVelocityVoltage flywheelVelocityRequest;
    private final VoltageOut voltageRequest;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Turret");

    private final SysIdRoutine turretSysIdRoutine;
    private final SysIdRoutine flywheelSysIdRoutine;

    /**
     * Creates a Turret subsystem with a turret motor (position control) and flywheel motor (velocity control).
     * Both motors use CTRE Pro licensing.
     * 
     * @param turretMotorId The CAN ID for the turret motor
     * @param flywheelMotorId The CAN ID for the flywheel motor
     * @param canbus The CANbus name
     */
    public Turret(int turretMotorId, int flywheelMotorId, String canbus) {
        this.turretMotor = new TalonFX(turretMotorId, canbus);
        this.flywheelMotor = new TalonFX(flywheelMotorId, canbus);

        this.turretPositionRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);
        
        this.flywheelVelocityRequest = new MotionMagicVelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);

        this.voltageRequest = new VoltageOut(0);

        configureTurretMotor();
        configureFlywheelMotor();

        // Configure turret SysId routine
        turretSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(1),
                null,
                (state) -> SignalLogger.writeString("SysIdTurret_state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> turretMotor.setControl(voltageRequest.withOutput(volts.in(Volts))),
                null,
                this
            )
        );

        // Configure flywheel SysId routine
        flywheelSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(2),
                null,
                (state) -> SignalLogger.writeString("SysIdFlywheel_state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> flywheelMotor.setControl(voltageRequest.withOutput(volts.in(Volts))),
                null,
                this
            )
        );
    }

    private void configureTurretMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Feedback configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(Constants.TurretConstants.kTurretGearRatio)
            .withRotorToSensorRatio(1.0);
        config.Feedback = feedbackConfigs;

        // Slot 0: PID + Feedforward for magic motion position
        Slot0Configs slot0 = new Slot0Configs();
        slot0.withKP(Constants.TurretConstants.kTurretP)
            .withKI(Constants.TurretConstants.kTurretI)
            .withKD(Constants.TurretConstants.kTurretD)
            .withKS(Constants.TurretConstants.kTurretS)
            .withKV(Constants.TurretConstants.kTurretV)
            .withKA(Constants.TurretConstants.kTurretA);
        config.Slot0 = slot0;

        // Motion Magic configuration for turret
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(Constants.TurretConstants.kTurretMMCruiseVelocity)
            .withMotionMagicAcceleration(Constants.TurretConstants.kTurretMMAcceleration)
            .withMotionMagicJerk(0);
        config.MotionMagic = motionMagicConfigs;

        // Motor settings
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);

        // Current limits
        config.CurrentLimits.withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(100)
            .withStatorCurrentLimitEnable(true);

        // Apply configuration
        turretMotor.getConfigurator().apply(config);

        // Update frequencies
        turretMotor.getPosition().setUpdateFrequency(250);
        turretMotor.getVelocity().setUpdateFrequency(250);
        turretMotor.optimizeBusUtilization();
    }

    private void configureFlywheelMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Feedback configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(Constants.TurretConstants.kFlywheelGearRatio)
            .withRotorToSensorRatio(1.0);
        config.Feedback = feedbackConfigs;

        // Slot 0: PID + Feedforward for magic motion velocity
        Slot0Configs slot0 = new Slot0Configs();
        slot0.withKP(Constants.TurretConstants.kFlywheelP)
            .withKI(Constants.TurretConstants.kFlywheelI)
            .withKD(Constants.TurretConstants.kFlywheelD)
            .withKS(Constants.TurretConstants.kFlywheelS)
            .withKV(Constants.TurretConstants.kFlywheelV)
            .withKA(Constants.TurretConstants.kFlywheelA);
        config.Slot0 = slot0;

        // Motion Magic configuration for flywheel
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(Constants.TurretConstants.kFlywheelMMCruiseVelocity)
            .withMotionMagicAcceleration(Constants.TurretConstants.kFlywheelMMAcceleration)
            .withMotionMagicJerk(0);
        config.MotionMagic = motionMagicConfigs;

        // Motor settings
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive);

        // Current limits
        config.CurrentLimits.withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

        // Apply configuration
        flywheelMotor.getConfigurator().apply(config);

        // Update frequencies
        flywheelMotor.getVelocity().setUpdateFrequency(250);
        flywheelMotor.optimizeBusUtilization();
    }

    /**
     * Sets the turret target position using magic motion profiling.
     * Position is clamped between min and max constants.
     * 
     * @param positionRotations Target position in rotations (mechanism frame)
     */
    public void setTurretPosition(double positionRotations) {
        double clampedPosition = Math.max(Constants.TurretConstants.kTurretMinPosition, 
                                          Math.min(Constants.TurretConstants.kTurretMaxPosition, positionRotations));
        turretMotor.setControl(turretPositionRequest.withPosition(clampedPosition));
    }

    /**
     * Gets the current turret position.
     * 
     * @return Current position in rotations (mechanism frame)
     */
    public double getTurretPosition() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    /**
     * Gets the current turret velocity.
     * 
     * @return Current velocity in rotations per second
     */
    public double getTurretVelocity() {
        return turretMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Sets the flywheel target velocity using motion magic velocity control.
     * 
     * @param velocityRotationsPerSecond Target velocity in rotations per second
     */
    public void setFlywheelVelocity(double velocityRotationsPerSecond) {
        flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(velocityRotationsPerSecond));
    }

    /**
     * Gets the current flywheel velocity.
     * 
     * @return Current velocity in rotations per second
     */
    public double getFlywheelVelocity() {
        return flywheelMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        turretMotor.stopMotor();
        flywheelMotor.stopMotor();
    }

    /**
     * Creates a command to set the turret position.
     * 
     * @param positionRotations Target position in rotations
     * @return Command that sets the turret position
     */
    public Command setTurretPositionCommand(double positionRotations) {
        return this.run(() -> this.setTurretPosition(positionRotations))
            .until(() -> Math.abs(getTurretPosition() - positionRotations) < Constants.TurretConstants.kTurretPositionTolerance)
            .withName("SetTurretPosition");
    }

    /**
     * Creates a command to set the flywheel velocity.
     * 
     * @param velocityRotationsPerSecond Target velocity in rotations per second
     * @return Command that sets the flywheel velocity
     */
    public Command setFlywheelVelocityCommand(double velocityRotationsPerSecond) {
        return this.run(() -> this.setFlywheelVelocity(velocityRotationsPerSecond))
            .withName("SetFlywheelVelocity");
    }

    /**
     * Updates network tables with current motor states.
     */
    @Override
    public void periodic() {
        // Turret telemetry
        tab.addNumber("Position", this::getTurretPosition);
        tab.addNumber("Velocity", this::getTurretVelocity);

        // Flywheel telemetry
        tab.addNumber("Flywheel Velocity", this::getFlywheelVelocity);
    }

    public Command turretSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return turretSysIdRoutine.quasistatic(direction);
    }

    public Command turretSysIdDynamic(SysIdRoutine.Direction direction) {
        return turretSysIdRoutine.dynamic(direction);
    }

    public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.quasistatic(direction);
    }

    public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.dynamic(direction);
    }
}
