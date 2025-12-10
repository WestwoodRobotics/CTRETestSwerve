package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{

    private final TalonFX motor;
    private final MotionMagicTorqueCurrentFOC motionMagic;
    private final VoltageOut voltageControl;
    private final SysIdRoutine sysIdRoutine;

    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    private GenericEntry posEntry, velEntry, accelEntry;
    

    public Arm(int deviceId, String canBus) {

        motor = new TalonFX(deviceId, canBus);
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotionMagic.MotionMagicCruiseVelocity = 1;
        configs.MotionMagic.MotionMagicAcceleration = 1;

        configs.Feedback.SensorToMechanismRatio = 18.0;


        configs.Slot0.kS = ArmConstants.kS;
        configs.Slot0.kG = ArmConstants.kG;
        configs.Slot0.kV = ArmConstants.kV;
        configs.Slot0.kA = ArmConstants.kA;

        configs.Slot0.kP = ArmConstants.kP;
        configs.Slot0.kI = ArmConstants.kI;
        configs.Slot0.kD = ArmConstants.kD;

        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(configs);

        motor.setPosition(ArmConstants.kZeroOffsetRotations);

        motor.getPosition().setUpdateFrequency(100);
        motor.getVelocity().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(50.0);
        motor.getAcceleration().setUpdateFrequency(100);
        motor.optimizeBusUtilization();

        motionMagic = new MotionMagicTorqueCurrentFOC(ArmConstants.kZeroOffsetRotations);
        voltageControl = new VoltageOut(0).withEnableFOC(true);

        sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.25).per(Second),        // Use default ramp rate (1 V/s)
            Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            (state) -> SignalLogger.writeString("SysIdArm_state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
           (Voltage volts) -> motor.setVoltage(volts.in(Volts)),
           null,
           this)
        );

        posEntry = tab.add("Position Rot", 0).getEntry();
        velEntry = tab.add("Velocity", 0).getEntry();
        accelEntry = tab.add("Acceleration", 0).getEntry();
    }

    public void periodic() {
        posEntry.setDouble(motor.getPosition().getValueAsDouble());
        velEntry.setDouble(motor.getVelocity().getValueAsDouble());
        accelEntry.setDouble(motor.getAcceleration().getValueAsDouble());
    }

    public void setPosition(double positionRotations) {
        if(positionRotations < ArmConstants.kMinPositionRotations) {
            positionRotations = ArmConstants.kMinPositionRotations;
        } else if(positionRotations > ArmConstants.kMaxPositionRotations) {
            positionRotations = ArmConstants.kMaxPositionRotations;
        }
        motor.setControl(motionMagic.withPosition(positionRotations));
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public void setVoltage(double volts) {
        // if((motor.getPosition().getValueAsDouble() >= ArmConstants.kMaxPositionRotations && volts > 0) || 
        //    (motor.getPosition().getValueAsDouble() <= ArmConstants.kMinPositionRotations && volts < 0)) {
        //     volts = 0.0; // Stop at limit
        // }
        motor.setControl(voltageControl.withOutput(volts));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
        
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
