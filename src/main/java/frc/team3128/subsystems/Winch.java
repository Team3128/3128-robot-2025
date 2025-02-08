package frc.team3128.subsystems;
import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static frc.team3128.Constants.IntakeConstants.currentThreshold;
import static frc.team3128.Constants.IntakeConstants.motor2;

public class Winch extends VoltageSubsystemBase {
    private static Winch instance;

    private Winch(){
        super(currentThreshold, motor2);
        configMotors();
    }

    public static synchronized Winch getInstance() {
        if (instance == null) instance = new Winch();
        return instance;
    }

    public NAR_Motor getMotor1() {
        return motor2;
    }
    @Override
    public void configMotors() {
        // IntakeConstans.motor1
        motor2.setInverted(false);
        motor2.setStatorLimit(40);
        motor2.enableVoltageCompensation(12);
    }
    @Override
    public Command reset() {
        return none();
    }
    @Override
    public Command run() {
        return none();
    }
    @Override
    public Command runVolts() {
        return none();
    }
    @Override
    public void initShuffleboard() {
    }
}
