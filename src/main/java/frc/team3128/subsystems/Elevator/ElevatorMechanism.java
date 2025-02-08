package frc.team3128.subsystems.Elevator;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import frc.team3128.doglog.DogLog;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.ElevatorConstants.*;

public class ElevatorMechanism extends PositionSubsystemBase {

    private static PIDFFConfig config = new PIDFFConfig(1, 1, 1);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX leader = new NAR_TalonFX(ELEVATOR_LEADER_ID);

    public ElevatorMechanism() {
        super(controller, leader);
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = new MotorConfig(
        ELEVATOR_GEAR_RATIO, 
        ELEVATOR_SAMPLE_PER_MINUTE,
        ELEVATOR_STATOR_CURRENT_LIMIT,
        ELEVATOR_INVERT,
        ELEVATOR_NEUTRAL_MODE,
        ELEVATOR_STATUS_FRAME);

        leader.configMotor(motorConfig);
    }

    @Override
    protected void configController() {
       controller.setInputRange(ELEVATOR_POSITION_MIN, ELEVATOR_POSITION_MAX);
       controller.configureFeedback(leader);
       controller.setTolerance(ELEVATOR_TOLERANCE);
    }   

    public void dogLogPeriodic(){
        DogLog.log(getName() + "Velocity", leader.getVelocity());
        DogLog.log(getName() + "Motor" , leader.getMotor());
        DogLog.log(getName() + "Position", leader.getPosition());
        DogLog.log(getName() + "StallCurrent", leader.getStallCurrent());
        DogLog.log(getName() + "State", leader.getState());
        DogLog.log(getName() + "Temperature", leader.getTemperature());
        DogLog.log(getName() + "At setpoint", controller.atSetpoint());
        DogLog.log(getName() + "Measurement", controller.getMeasurement());
    }
}