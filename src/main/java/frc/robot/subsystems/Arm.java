package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private TalonFX armMotor;
    private Slot0Configs configs;  
    private MotionMagicConfigs motionMagicConfigs;  
    public Arm() {
        armMotor = new TalonFX(0, "SwerveCAN"); // Replace 0 with the actual CAN ID

        configs = new Slot0Configs();
        configs.kP = 1.0;
        configs.kI = 0.0;
        configs.kD = 0.1;
        configs.kS = 0.0;
        configs.kV = 0.0;
        configs.kG = 0.0;
        armMotor.getConfigurator().apply(configs);

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 15; // Set appropriate cruise velocity
        motionMagicConfigs.MotionMagicAcceleration = 80; // Set appropriate acceleration

        armMotor.getConfigurator().apply(motionMagicConfigs);
        

    }

    public void setArmPosition(double position) {
        MotionMagicVoltage request = new MotionMagicVoltage(position).withSlot(0);
        armMotor.setControl(request);
    }
}
