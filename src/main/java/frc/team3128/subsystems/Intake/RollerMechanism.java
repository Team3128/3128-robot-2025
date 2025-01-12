package frc.team3128.subsystems.Intake;

import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.utility.shuffleboard.NAR_Shuffleboard;

import static frc.team3128.Constants.IntakeConstants.*;

public class RollerMechanism extends VoltageSubsystemBase {

    protected static NAR_TalonFX leader = new NAR_TalonFX(ROLLER_LEADER_ID);
    protected static DigitalInput firstSensor = new DigitalInput(FIRST_SENSOR_ID);

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
        return firstSensor.get();
    }

	@Override
	public Command reset() {
        return null;
	}

	@Override
	public void initShuffleboard() {
        
	}
}