package frc.robot.utils;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.CandleConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Candle {
    
    private CANdle candle;
    private CommandSwerveDrivetrain drivetrain;
    CANdleConfiguration cfg;

    public Candle(int deviceId, CANBus canbus, CommandSwerveDrivetrain drivetrain) {
        this.candle = new CANdle(deviceId, canbus);

        cfg = new CANdleConfiguration();
        cfg.LED.BrightnessScalar = 1.0;
        cfg.LED.StripType = StripTypeValue.GRB;
        candle.getConfigurator().apply(cfg);

        for (int i = 0; i < 8; i++){
            candle.setControl(new EmptyAnimation(i));
        }

        drivetrain.setCANdle(candle);
    }

    public void setColor(Color color) {
        candle.setControl(new SolidColor(CandleConstants.startIndex, CandleConstants.endIndex)
            .withColor(new RGBWColor(color).scaleBrightness(1)));
    }

    public void setColor(Color color, double brightness) {
        candle.setControl(new SolidColor(CandleConstants.startIndex, CandleConstants.endIndex)
            .withColor(new RGBWColor(color).scaleBrightness(brightness)));
    }

    public void setColor(double r, double b, double g) {
        candle.setControl(new SolidColor(CandleConstants.startIndex, CandleConstants.endIndex)
            .withColor(new RGBWColor(new Color(r, g, b)).scaleBrightness(1)));
    }

    public void setColor(double r, double b, double g, double brightness) {
        candle.setControl(new SolidColor(CandleConstants.startIndex, CandleConstants.endIndex)
            .withColor(new RGBWColor(new Color(r, g, b)).scaleBrightness(brightness)));
    }

    public void turnOff() {
        candle.setControl(new SolidColor(CandleConstants.startIndex, CandleConstants.endIndex)
            .withColor(new RGBWColor(new Color(0, 0, 0)).scaleBrightness(1)));
    }
}
