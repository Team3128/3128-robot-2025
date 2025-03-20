package frc.team3128.subsystems.Led;

import static frc.team3128.Constants.SwerveConstants.DRIVETRAIN_CANBUS_NAME;
import static frc.team3128.Constants.LedConstants.*;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.led.CANdleConfiguration;

public class LedMechanism {
    
    private static LedMechanism instance;

    private final CANdle candle = new CANdle(CANDLE_ID, DRIVETRAIN_CANBUS_NAME);

    public static synchronized LedMechanism getInstance() {
        if (instance == null) {
            instance = new LedMechanism();
        }

        return instance;
    }

    public LedMechanism() {
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
        candle.setLEDs(0,0,0, WHITE_VALUE, STARTING_ID, NUM_LED);
        candle.animate(null, 0);
    }

    public void setColor(Color color) {
        setColor(color, 0);
    }

    public void setColor(Color color, double magnitude) {
        reset();
        candle.setLEDs((int)color.red, (int)color.green, (int)color.blue, 0, STARTING_ID, (int)Math.ceil(MathUtil.clamp(magnitude, 0, 1) * NUM_LED));
    }

    public void setColor(Color color, int white) {
        reset();
        candle.setLEDs((int)color.red, (int)color.green, (int)color.blue, white, STARTING_ID, NUM_LED);
    }

    public void setAnimation(Animation animation) {
        reset();
        candle.animate(animation);
    }
}
