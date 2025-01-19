package frc.team3128.subsystems.Elevator;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.ElevatorConstants.*;

public class ElevatorMechanism extends PositionSubsystemBase {

    private static PIDFFConfig config = new PIDFFConfig(0.1, 0, 0);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_CANSpark leader = new NAR_CANSpark(ELEVATOR_LEADER_ID);
    protected static NAR_CANSpark follower = new NAR_CANSpark(ELEVATOR_FOLLOWER_ID);

    public ElevatorMechanism() {
        super(controller, leader, follower);
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

        //TODO: fix follwoer config

        follower.configMotor(motorConfig.invertFollower());
        // follower.follow(leader);

    }

    @Override
    protected void configController() {
       controller.setInputRange(ELEVATOR_POSITION_MIN, ELEVATOR_POSITION_MAX);
       controller.configureFeedback(leader);
       controller.setTolerance(ELEVATOR_TOLERANCE);
    }   
}