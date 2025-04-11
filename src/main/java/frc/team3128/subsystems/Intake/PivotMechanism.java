package frc.team3128.subsystems.Intake;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.IntakeConstants.*;

public class PivotMechanism extends PositionSubsystemBase {

    public static PivotMechanism instance;

    // private static PIDFFConfig config = new PIDFFConfig(0.165, 0, 0, 0.26778, 0.01694, 0.0, 0.0);
    private static PIDFFConfig config = new PIDFFConfig(0.16, 0, 0, 0.23783, 0.01558, 0.00234, 0.0);

    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_CANSpark leader = new NAR_CANSpark(PIVOT_LEADER_ID, ControllerType.CAN_SPARK_FLEX);

    private PivotMechanism() {
        super(controller, leader);
        initShuffleboard();
    }

    public static PivotMechanism getInstance() {
        if (instance == null) {
            instance = new PivotMechanism();
        }

        return instance;
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = new MotorConfig(
        PIVOT_GEAR_RATIO, 
        PIVOT_SAMPLE_PER_MINUTE,
        PIVOT_STATOR_CURRENT_LIMIT,
        PIVOT_INVERT,
        PIVOT_NEUTRAL_MODE,
        PIVOT_STATUS_FRAME);

        leader.configMotor(motorConfig);
    }

    @Override
    protected void configController() {
       controller.setInputRange(PIVOT_POSITION_MIN, PIVOT_POSITION_MAX);
       controller.configureFeedback(leader);
       controller.setTolerance(PIVOT_TOLERANCE);
    }   
}