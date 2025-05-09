package frc.team3128.subsystems.Elevator;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.ElevatorConstants.*;

public class ElevatorMechanism extends PositionSubsystemBase {

    private static ElevatorMechanism instance;
    //30, 0, 0, 0.25086, 4.52908, 0.99630, 0
    private static PIDFFConfig config = new PIDFFConfig(20, 0, 0, 0.6, 2.91916, 0.67429, 0.3);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_CANSpark left = new NAR_CANSpark(ELEVATOR_LEFT_ID, ControllerType.CAN_SPARK_FLEX);
    protected static NAR_CANSpark right = new NAR_CANSpark(ELEVATOR_RIGHT_ID, ControllerType.CAN_SPARK_FLEX);

    private ElevatorMechanism() {
        super(controller, left, right);
    }

    public static synchronized ElevatorMechanism getInstance() {
        if (instance == null) instance = new ElevatorMechanism();
        return instance;
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
       addDisableCondition(()-> ((getPosition() >= 0.90 * ELEVATOR_POSITION_MAX) && (getVolts() > 6) && (getVelocity() > 0))); // if too close to top and going towards top and getting too much power then disable
    }   
}