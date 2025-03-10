package frc.team3128.subsystems.Climber;

import static frc.team3128.Constants.ClimberConstants.*;

import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import frc.team3128.doglog.DogLog;

public class RollerMechanism extends VoltageSubsystemBase  {

    public static RollerMechanism instance;

    protected static NAR_CANSpark leader = new NAR_CANSpark(CLIMB_ROLLER_ID, ControllerType.CAN_SPARK_FLEX);

    private RollerMechanism() {
        super(leader);
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
        CLIMB_ROLLER_GEAR_RATIO, 
        CLIMB_ROLLER_SAMPLE_PER_MINUTE,
        CLIMB_ROLLER_STATOR_CURRENT_LIMIT,
        CLIMB_ROLLER_INVERT,
        CLIMB_ROLLER_NEUTRAL_MODE);

        leader.configMotor(motorConfig);
    }

    @Override
 	public void initShuffleboard() {

 	}

    public void dogLogPeriodic(){
        DogLog.log(getName() + "Velocity", leader.getVelocity());
        DogLog.log(getName() + "Position", leader.getPosition());
        DogLog.log(getName() + "Applied Output", leader.getAppliedOutput());
        DogLog.log(getName() + "State", leader.getState());
    }
}
