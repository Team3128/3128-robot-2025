package frc.team3128.subsystems.Leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.led.CANdleConfiguration;

import static frc.team3128.Constants.LedsConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;

public class LedsMechanism {
    
    private static LedsMechanism instance;

    private final CANdle candle = new CANdle(CANDLE_ID, DRIVETRAIN_CANBUS_NAME);

    public static synchronized LedsMechanism getInstance() {
        if (instance == null)
            instance = new LedsMechanism();
        return instance;
    }

    public LedsMechanism() {
        configCandle();
    }

    private void configCandle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.disableWhenLOS = true; // turn off when signal lost
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }

    public void reset() {
        candle.setLEDs(0,0,0, 0, 0, MAX_HEIGHT);
        candle.animate(null, 0);
    }

    public void setColor(Color color) {
        setColor(color, MAX_HEIGHT);
    }

    public void setColor(Color color, int height) {
        reset();
        candle.setLEDs((int) (color.red * 256), (int) (color.green * 256), (int) (color.blue * 256), 0, 0, height);
    }

    public void setAnimation(Animation animation) {
        reset();
        candle.animate(animation, 0);
    }
}
