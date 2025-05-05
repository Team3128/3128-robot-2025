package frc.team3128.subsystems.Leds;

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

    public void set(Color color, int height) {
        candle.setLEDs((int) color.red, (int) color.green, (int) color.blue, 0, 0, height);
    }

    public void set(Color color) {
        candle.setLEDs((int) color.red, (int) color.green, (int) color.blue, 0, 0, MAX_HEIGHT);
    }

}
