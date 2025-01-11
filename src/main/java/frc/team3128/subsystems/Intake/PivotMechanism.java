package frc.team3128.subsystems.Intake;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.IntakeConstants.*;

public class PivotMechanism extends PositionSubsystemBase {

    private static PIDFFConfig config = new PIDFFConfig(1, 1, 1);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX leader = new NAR_TalonFX(PIVOT_LEADER_ID);

    public PivotMechanism() {
        super(controller, leader);
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