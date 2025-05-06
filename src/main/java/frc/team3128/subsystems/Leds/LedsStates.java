package frc.team3128.subsystems.Leds;


import edu.wpi.first.wpilibj.util.Color;

public enum LedsStates {
    DISABLED(Color.kGreenYellow),
    ZEROED(Color.kGreen),
    NEUTRAL(Color.kWhite),
    CLIMB_PRIME(Color.kBlue),
    CLIMB(Color.kViolet);

    private Color color;

   
    private LedsStates(int r, int g, int b) {
        this.color = new Color(r, g, b);
    }

    private LedsStates(Color color) {
        this.color = color;
    }

    public Color getColor() {
        return this.color;
    }


}