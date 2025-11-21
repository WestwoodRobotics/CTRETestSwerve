package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    private final TalonFX motor;
    private final MotionMagicTorqueCurrentFOC mmRequest;

    public Arm(int deviceId, CANBus canbus, int cancoderId) {

        motor = new TalonFX(deviceId, canbus);
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotionMagic.MotionMagicCruiseVelocity = 1;
        configs.MotionMagic.MotionMagicAcceleration = 0.5;

        configs.Slot0.kS = 0.1;
        configs.Slot0.kV = 0.1;
        configs.Slot0.kA = 0.1;

        configs.Slot0.kP = 0.1;
        configs.Slot0.kI = 0.1;
        configs.Slot0.kD = 0.1;
        
        configs.Feedback.FeedbackRemoteSensorID = cancoderId;
        configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(configs);

        motor.getPosition().setUpdateFrequency(100);
        motor.getVelocity().setUpdateFrequency(100);
        motor.optimizeBusUtilization();

        mmRequest = new MotionMagicTorqueCurrentFOC(0);
    }

    public void setPosition(double positionRotations) {
        motor.setControl(mmRequest.withPosition(positionRotations));
    }
}
