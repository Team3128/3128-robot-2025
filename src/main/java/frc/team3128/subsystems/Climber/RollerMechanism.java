package frc.team3128.subsystems.Climber;

import static frc.team3128.Constants.ClimberConstants.CLIMB_ROLLER_GEAR_RATIO;
import static frc.team3128.Constants.ClimberConstants.CLIMB_ROLLER_ID;
import static frc.team3128.Constants.ClimberConstants.CLIMB_ROLLER_INVERT;
import static frc.team3128.Constants.ClimberConstants.CLIMB_ROLLER_NEUTRAL_MODE;
import static frc.team3128.Constants.ClimberConstants.CLIMB_ROLLER_SAMPLE_PER_MINUTE;
import static frc.team3128.Constants.ClimberConstants.CLIMB_ROLLER_STATOR_CURRENT_LIMIT;

import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;

public class RollerMechanism extends VoltageSubsystemBase  {

    protected static NAR_CANSpark leader = new NAR_CANSpark(CLIMB_ROLLER_ID);

    public RollerMechanism() {
        super(leader);
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
}
