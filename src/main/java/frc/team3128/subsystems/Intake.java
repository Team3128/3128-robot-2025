package frc.team3128.subsystems;
import common.core.subsystems.VoltageSubsystemBase;
import common.core.controllers.Controller.Type;
import common.hardware.motorcontroller.NAR_CANSPark;
import common.hardware.motorcontroller.NAR_CANSPark.ControllerType;
import Constants.IntakeConstants.*;

public class Intake extends VoltageSubsystemBase {

    public NAR_CANSPark rollerMotor;

    public Intake(CURRENT_THRESHOLD, rollerMotor) {

    }

    private void configMotors(){
        rollerMotor = new NAR_CANSPark(ROLLER_ID, )
    }
}
