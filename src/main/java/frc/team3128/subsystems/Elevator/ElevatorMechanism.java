package frc.team3128.subsystems.Elevator;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.ElevatorConstants.*;

public class ElevatorMechanism extends PositionSubsystemBase {

    private static PIDFFConfig config = new PIDFFConfig(64, 0, 0, 0.22086, 4.52908,0.99630,0);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_CANSpark left = new NAR_CANSpark(ELEVATOR_LEFT_ID, ControllerType.CAN_SPARK_FLEX);
    protected static NAR_CANSpark right = new NAR_CANSpark(ELEVATOR_RIGHT_ID, ControllerType.CAN_SPARK_FLEX);

    public ElevatorMechanism() {
        super(controller, left, right);
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


        left.configMotor(motorConfig);
        right.configMotor(motorConfig.follower());

        initShuffleboard();

    }

    @Override
    protected void configController() {
       controller.setInputRange(ELEVATOR_POSITION_MIN, ELEVATOR_POSITION_MAX);
       controller.configureFeedback(left);
       controller.setTolerance(ELEVATOR_TOLERANCE);
    }   
}