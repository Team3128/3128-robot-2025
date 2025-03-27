package frc.team3128.subsystems.Led;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.util.Color;

public enum LedStates {

    UNDEFINED(0, 0, 0),

    DISABLED(Color.kOrange),

    DEFAULT(Color.kWhite), //Value irrelavent

    CLIMB_PRIME(Color.kOrange),
    CLIMB(Color.kGreen),

    AUTO_HOLD(Color.kPink),

    INTAKE(Color.kBlue),
    EJECT_OUTTAKE(Color.kPurple),

    PRIME(Color.kOrange),
    SCORE(Color.kGreen),

    ODD_REEF(Color.kPurple),
    EVEN_REEF(Color.kYellow),
    
    SOURCE(Color.kRed);

    private Color color;
    private Animation animation;

    private LedStates(Animation animation) {
        this.animation = animation;
    }

    private LedStates(int r, int g, int b) {
        this.color = new Color(r, g, b);
    }

    private LedStates(Color color) {
        this.color = color;
    }

    public Color getColor() {
        return this.color;
    }

    public Animation getAnimation() {
        return this.animation;
    }

}