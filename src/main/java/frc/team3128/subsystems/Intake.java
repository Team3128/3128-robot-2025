package frc.team3128.subsystems;
import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_Motor;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static frc.team3128.Constants.IntakeConstants.currentThreshold;
import static frc.team3128.Constants.IntakeConstants.motor1;
import static frc.team3128.Constants.IntakeConstants.motor2;

public class Intake extends VoltageSubsystemBase {
    private static Intake instance;

    private Intake(){
        super(currentThreshold, motor1, motor2);
        configMotors();
    }

    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    public NAR_Motor getMotor1() {
        return motor1;
    }

    public NAR_Motor getMotor2() {
        return motor2;
    }
    
    @Override
    public void configMotors() {
        // IntakeConstans.motor1
        motor1.setInverted(false);
        motor1.setStatorLimit(40);
        motor1.enableVoltageCompensation(12);
        //CHECK : Should moter1 and moter2 should be configured together or separate?

        // IntakeConstans.motor2
        motor2.setInverted(false);
        motor2.setStatorLimit(40);
        motor2.enableVoltageCompensation(12);
    }
    
    public void configMotor1() {
        // IntakeConstans.motor1
        motor1.setInverted(false);
        motor1.setStatorLimit(40);
        motor1.enableVoltageCompensation(12);
    }

    public void configMotor2() {
        // IntakeConstans.motor2
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
