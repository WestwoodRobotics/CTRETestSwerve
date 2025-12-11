package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX motor;
    private final MotionMagicTorqueCurrentFOC magicTorqueRequest;
    private final MotionMagicVelocityVoltage magicVelocityRequest;
    private final VoltageOut voltageRequest;
    private final SysIdRoutine sysIdRoutine;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    private GenericEntry posEntry, velEntry, accelEntry;

    /**
     * Creates an Arm subsystem with a TalonFX motor using magic motion profiling.
     * 
     * @param canbus The CANbus name
     * @param motorId The motor CAN ID
     */
    public Arm(int motorId, String canbus) {
        this.motor = new TalonFX(motorId, canbus);
        this.magicTorqueRequest = new MotionMagicTorqueCurrentFOC(0)
            .withSlot(0)
            .withFeedForward(0);
        this.magicVelocityRequest = new MotionMagicVelocityVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);
        this.voltageRequest = new VoltageOut(0);

        configureMotor();

        // Configure SysId routine
        this.sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, null),
            new SysIdRoutine.Mechanism(
                voltage -> motor.setControl(voltageRequest.withOutput(voltage.in(edu.wpi.first.units.Units.Volts))),
                (state) -> SignalLogger.writeString("arm_state", state.toString()),
                this
            )
        );

        posEntry = tab.add("Position Rot", 0).getEntry();
        velEntry = tab.add("Velocity", 0).getEntry();
        accelEntry = tab.add("Acceleration", 0).getEntry();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor feedback configuration
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RotorSensor)
            .withSensorToMechanismRatio(Constants.ArmConstants.kGearRatio)
            .withRotorToSensorRatio(1.0);
        config.Feedback = feedbackConfigs;

        // Set initial position offset
        motor.setPosition(Constants.ArmConstants.kZeroOffsetRotations);

        // Slot 0: PID + Feedforward for magic motion
        Slot0Configs slot0 = new Slot0Configs();
        slot0.withKP(Constants.ArmConstants.kP)
            .withKI(Constants.ArmConstants.kI)
            .withKD(Constants.ArmConstants.kD)
            .withKS(Constants.ArmConstants.kS)
            .withKV(Constants.ArmConstants.kV)
            .withKA(Constants.ArmConstants.kA)
            .withKG(Constants.ArmConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine);
        config.Slot0 = slot0;

        // Magic motion configuration
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicCruiseVelocity(Constants.ArmConstants.kMMCruiseVelocity)
            .withMotionMagicAcceleration(Constants.ArmConstants.kMMAcceleration)
            .withMotionMagicJerk(0); // Instantaneous jerk
        config.MotionMagic = motionMagicConfigs;

        // Motor settings
        config.MotorOutput.withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);

        // Current limits
        config.CurrentLimits.withSupplyCurrentLimit(70)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

        // Apply configuration
        motor.getConfigurator().apply(config);

        // Max out update frequencies for critical signals
        motor.getPosition().setUpdateFrequency(250);
        motor.getVelocity().setUpdateFrequency(250);
        motor.getAcceleration().setUpdateFrequency(250);
        motor.getMotorVoltage().setUpdateFrequency(250);
        motor.getSupplyCurrent().setUpdateFrequency(250);
        motor.getStatorCurrent().setUpdateFrequency(250);
        motor.optimizeBusUtilization();
    }

    /**
     * Sets the arm target position using magic motion profiling.
     * 
     * @param positionRotations Target position in rotations (mechanism frame)
     */
    public void setPosition(double positionRotations) {
        // Clamp position to limits
        double clampedPosition = Math.max(
            Constants.ArmConstants.kMinPositionRotations,
            Math.min(Constants.ArmConstants.kMaxPositionRotations, positionRotations)
        );

        motor.setControl(magicTorqueRequest.withPosition(clampedPosition));
    }

    /**
     * Gets the current arm position.
     * 
     * @return Current position in rotations (mechanism frame)
     */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * Gets the current arm velocity.
     * 
     * @return Current velocity in rotations per second
     */
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * Sets the arm target velocity using motion magic velocity control.
     * 
     * @param velocityRotationsPerSecond Target velocity in rotations per second
     */
    public void setVelocity(double velocityRotationsPerSecond) {
        motor.setControl(magicVelocityRequest.withVelocity(velocityRotationsPerSecond));
    }

    /**
     * Stops the arm motor.
     */
    public void stop() {
        motor.stopMotor();
    }

    /**
     * Updates network tables and refreshes signals.
     */
    @Override
    public void periodic() {
        posEntry.setDouble(this.getPosition());
        velEntry.setDouble(this.getVelocity());
        accelEntry.setDouble(motor.getAcceleration().getValueAsDouble());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
        
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}