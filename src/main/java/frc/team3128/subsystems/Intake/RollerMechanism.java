package frc.team3128.subsystems.Intake;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.IntakeConstants.*;

public class RollerMechanism extends PositionSubsystemBase {

    private static PIDFFConfig config = new PIDFFConfig(1, 1, 1);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX leader = new NAR_TalonFX(ROLLER_LEADER_ID);

    public RollerMechanism() {
        super(controller, leader);
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
    protected void configController() {
       controller.setInputRange(ROLLER_POSITION_MIN, ROLLER_POSITION_MAX);
       controller.configureFeedback(leader);
       controller.setTolerance(ROLLER_TOLERANCE);
    }

    public boolean hasObjectPresent() {
        // TODO Auto-generated method stub
        return true;
    }
}