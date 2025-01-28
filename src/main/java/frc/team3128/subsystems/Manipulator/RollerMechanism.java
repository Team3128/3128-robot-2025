package frc.team3128.subsystems.Manipulator;

import common.core.subsystems.VoltageSubsystemBase;
import common.doglog.DogLog;
import common.hardware.motorcontroller.NAR_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.utility.shuffleboard.NAR_Shuffleboard;

import static frc.team3128.Constants.ManipulatorConstants.*;

public class RollerMechanism extends VoltageSubsystemBase {

    protected static NAR_TalonFX leader = new NAR_TalonFX(ROLLER_LEADER_ID);
    // protected static DigitalInput firstSensor = new DigitalInput(FIRST_SENSOR_ID);
    // protected static DigitalInput secondSensor = new DigitalInput(SECOND_SENSOR_ID);

    public RollerMechanism() {
        super(leader);
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
        
	}

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public Command resetCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetCommand'");
    }
    public void dogLogPeriodic(){
        DogLog.log(getName() + "Velocity", leader.getVelocity());
        DogLog.log(getName() + "Motor" , leader.getMotor());
        DogLog.log(getName() + "Position", leader.getPosition());
        DogLog.log(getName() + "StallCurrent", leader.getStallCurrent());
        DogLog.log(getName() + "State", leader.getState());
        DogLog.log(getName() + "Temperature", leader.getTemperature());
    }
}