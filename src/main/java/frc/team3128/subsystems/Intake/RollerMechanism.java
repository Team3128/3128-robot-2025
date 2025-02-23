package frc.team3128.subsystems.Intake;

import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import edu.wpi.first.wpilibj2.command.Command;
 import common.hardware.motorcontroller.NAR_CANSpark;
 import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
 import common.utility.shuffleboard.NAR_Shuffleboard;

 import static frc.team3128.Constants.IntakeConstants.*;

 public class RollerMechanism extends VoltageSubsystemBase {

    public static RollerMechanism instance;

     protected static NAR_CANSpark leader = new NAR_CANSpark(ROLLER_LEADER_ID, ControllerType.CAN_SPARK_FLEX);

     private RollerMechanism() {
        super(leader);
        // initShuffleboard();
     }

    public static RollerMechanism getInstance() {
        if (instance == null) {
            instance = new RollerMechanism();
        }

        return instance;
    }

     @Override
     protected void configMotors() {
         MotorConfig motorConfig = new MotorConfig(
         ROLLER_GEAR_RATIO, 
         ROLLER_SAMPLE_PER_MINUTE,
         ROLLER_STATOR_CURRENT_LIMIT,
         ROLLER_INVERT,
         ROLLER_NEUTRAL_MODE,
         ROLLER_STATUS_FRAME);

         leader.configMotor(motorConfig);
     }

     @Override
     public boolean hasObjectPresent() {
         // return firstSensor.get();
         return true;
     }

 	@Override
 	public Command resetCommand() {
        return null;
 	}

 	@Override
 	public void initShuffleboard() {
        NAR_Shuffleboard.addData("Intake Current", "Current", ()-> getCurrent(), 0, 0);
 	}
 }