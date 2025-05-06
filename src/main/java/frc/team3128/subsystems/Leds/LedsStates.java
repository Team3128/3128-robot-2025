package frc.team3128.subsystems.Leds;


import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.util.Color;

public enum LedsStates {
    DISABLED(Color.kGreenYellow),
    ZEROED(Color.kGreen),
    NEUTRAL(Color.kWhite),
    CLIMB_PRIME(Color.kBlue),
    CLIMB(Color.kViolet);

    private Color color;
    private Animation animation;
    private boolean isAnimation;
   
    private LedsStates(int r, int g, int b) {
        this.color = new Color(r, g, b);
    }

    private LedsStates(Color color) {
        this.color = color;
        this.animation = null;
        isAnimation = false;
    }

    private LedsStates(Animation animation) {
        this.color = Color.kBlack;
        this.animation = animation;
        isAnimation = true;
    }

    public Color getColor() {
        return this.color;
    }

    public boolean getIsAnimation() {
        return isAnimation;
    }

    public Animation getAnimation() {
        return animation;
    }
}