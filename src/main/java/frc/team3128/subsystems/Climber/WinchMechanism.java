package frc.team3128.subsystems.Climber;

import common.core.controllers.*;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.ClimberConstants.*;

public class WinchMechanism extends PositionSubsystemBase {

    public static WinchMechanism instance;

    private static PIDFFConfig config = new PIDFFConfig(0.00001, 0, 0, 12, 0, 0, 0);
    protected static ControllerBase controller = new Controller(config, Controller.Type.POSITION);

    public static NAR_CANSpark leader = new NAR_CANSpark(CLIMBER_WINCH_ID, ControllerType.CAN_SPARK_FLEX);

    private WinchMechanism() {
        super(controller, leader);
    }

    public static WinchMechanism getInstance() {
        if (instance == null) {
            instance = new WinchMechanism();
        }

        return instance;
    }

    @Override
    protected void configMotors() {
        MotorConfig motorConfig = new MotorConfig(
        CLIMBER_GEAR_RATIO, 
        CLIMBER_SAMPLE_PER_MINUTE,
        CLIMBER_STATOR_CURRENT_LIMIT,
        CLIMBER_INVERT,
        CLIMBER_NEUTRAL_MODE,
        CLIMBER_STATUS_FRAME);

        leader.configMotor(motorConfig);

        initShuffleboard();
    }

    @Override
    protected void configController() {
       controller.setInputRange(CLIMBER_POSITiON_MIN, CLIMBER_POSITION_MAX);
       controller.configureFeedback(leader);
       controller.setTolerance(CLIMBER_TOLERANCE);
    } 
}