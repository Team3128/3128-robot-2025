package frc.team3128.subsystems.Led;

import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.led.Animation;

import io.vavr.collection.List;

import static frc.team3128.Constants.LedConstants.*;

public enum LedStates {
    DISABLED(Colors.ORANGE),
    ENABLED(Colors.GREEN),
    PROXIMITY(Colors.BLUE),
    AUTO(Colors.RED);

    private Colors color;

    public static List<LedStates> functionalStates = List.of(DISABLED, ENABLED, PROXIMITY, AUTO);

    private LedStates(Colors color) {
        this.color = color;
    }

    public Colors getColor() {
        return this.color;
    }
}