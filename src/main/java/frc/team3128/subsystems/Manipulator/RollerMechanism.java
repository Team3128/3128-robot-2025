package frc.team3128.subsystems.Manipulator;
import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.utility.shuffleboard.NAR_Shuffleboard;

import static frc.team3128.Constants.ManipulatorConstants.*;

public class RollerMechanism extends VoltageSubsystemBase {

    private static RollerMechanism instance;

    public static synchronized RollerMechanism getInstance() {
        if(instance == null) instance = new RollerMechanism();
        return instance;
    }

    protected static NAR_CANSpark leader = new NAR_CANSpark(ROLLER_LEADER_ID, ControllerType.CAN_SPARK_FLEX);
    //protected static DigitalInput firstSensor = new DigitalInput(FIRST_SENSOR_ID);
    //protected static DigitalInput secondSensor = new DigitalInput(SECOND_SENSOR_ID);

    public RollerMechanism() {
        super(leader);
        // initShuffleboard();
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
        // return firstSensor.get() || secondSensor.get();
        return true;
    }


	@Override
	public void initShuffleboard() {
        NAR_Shuffleboard.addData(getName(), "Power", ()-> leader.getAppliedOutput(), 0, 0);
        NAR_Shuffleboard.addData(getName(), "Current", ()->leader.getStallCurrent(), 1, 0);
        NAR_Shuffleboard.addData(getName(), "Torque", ()-> leader.getTorque(), 2, 0);
	}
}