package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs; // added
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final TalonFX motor;
    private final MotionMagicVoltage magicPositionRequest;
    private final MotionMagicVelocityVoltage magicVelocityRequest;
    private final VoltageOut voltageRequest;
    private final SysIdRoutine sysIdRoutine;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    private GenericEntry posEntry, velEntry, accelEntry;

    // Max trackers
    private double maxVelRps = 0.0;
    private double maxAccelRps2 = 0.0;

    // For computed acceleration
    private double lastVelRps = 0.0;
    private double lastTimeSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    /**
     * Creates an Arm subsystem with a TalonFX motor using magic motion profiling.
     * 
     * @param canbus The CANbus name
     * @param motorId The motor CAN ID
     */
    public Arm(int motorId, String canbus) {
        this.motor = new TalonFX(motorId, canbus);
        this.magicPositionRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(true);
        this.magicVelocityRequest = new MotionMagicVelocityVoltage(0)
            .withSlot(1) // use feedforward-only slot for velocity
            .withEnableFOC(true);
        this.voltageRequest = new VoltageOut(0);

        configureMotor();

        // Configure SysId routine
        sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.25).per(Second),
            Volts.of(1),
            null,
            // Log state with SignalLogger class
            (state) -> SignalLogger.writeString("SysIdArm_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
           (volts) -> motor.setVoltage(volts.in(Volts)),
           null,
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

        // Slot 0: PID + Feedforward for magic motion position
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

        // Slot 1: Feedforward-only for velocity (PID gains zeroed)
        Slot1Configs slot1 = new Slot1Configs();
        slot1.withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(Constants.ArmConstants.kS)
            .withKV(Constants.ArmConstants.kV)
            .withKA(Constants.ArmConstants.kA)
            .withKG(Constants.ArmConstants.kG);
        config.Slot1 = slot1;

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

        motor.setControl(magicPositionRequest.withPosition(positionRotations));
    }

    public void resetPosition(double positionRotations) {
        motor.setPosition(positionRotations);
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
        // Current position/velocity
        double pos = this.getPosition();
        double vel = this.getVelocity();

        // Prefer device-reported acceleration; compute if unavailable
        double accel = motor.getAcceleration().getValueAsDouble();
        // Update trackers
        if (Math.abs(vel) > Math.abs(maxVelRps)) {
            maxVelRps = vel;
        }
        if (Math.abs(accel) > Math.abs(maxAccelRps2)) {
            maxAccelRps2 = accel;
        }

        // Publish to SmartDashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Arm/PositionRot", pos);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Arm/VelocityRps", vel);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Arm/AccelRps2", accel);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Arm/MaxVelocityRps", maxVelRps);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Arm/MaxAccelRps2", maxAccelRps2);

        // Keep Shuffleboard entries if desired
        posEntry.setDouble(pos);
        velEntry.setDouble(vel);
        accelEntry.setDouble(accel);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
        
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}