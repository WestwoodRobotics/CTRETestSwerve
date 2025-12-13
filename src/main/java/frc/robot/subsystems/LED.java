package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    public enum Mode{
        CAMERA,
        MANUAL
    }

    private Mode mode = Mode.CAMERA;
    public CANdle candle;
    public LED(int deviceID, String canBus){
        candle = new CANdle(deviceID, canBus);

        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 1.0;
        cfg.LED.StripType = StripTypeValue.GRB;

        candle.getConfigurator().apply(cfg);
    }

    public void setMode(Mode mode){
        this.mode = mode;
    }
    
    public Mode getMode(){
        return mode;
    }

    public void setSolidColor(Color color, double brightness){
        candle.setControl(new SolidColor(0,26).withColor(new RGBWColor(color).scaleBrightness(brightness)));
    }
    
    public void clearColor(){
        candle.setControl(new SolidColor(0,26).withColor(new RGBWColor(new Color(0,0,0)).scaleBrightness(1)));

    }

    public void cameraSetColor(Color color, double brightness){
        if (mode == Mode.CAMERA){
            setSolidColor(color, brightness);
        }
    }

    public void cameraClearColor(){
        if (mode == Mode.CAMERA){
            clearColor();
        }
    }

    
    public void startFireAnimation(){
        FireAnimation FIRE = new FireAnimation(0,27).withBrightness(1).withCooling(0.3);
        candle.setControl(FIRE);   
    }

}
