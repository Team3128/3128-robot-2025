package frc.team3128.subsystems.Manipulator;

public enum ManipulatorStates {
    IDLE,
    NEUTRAL,
    FORWARD(0.5),
    REVERSE(-0.5);

    private double power;

    public static ManipulatorStates[] functionalStates = {NEUTRAL, FORWARD, REVERSE};

    private ManipulatorStates(double power) {
        this.power = power;
    }

    private ManipulatorStates() {
        this.power = 0;
    }

    public double getPower() {
        return power;
    }
}
