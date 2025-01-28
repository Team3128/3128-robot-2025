package frc.team3128.subsystems.Climber;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.doglog.DogLog;
import common.hardware.motorcontroller.NAR_TalonFX;
import edu.wpi.first.wpilibj.PWM;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.ClimberConstants.*;

public class WinchMechanism extends PositionSubsystemBase {

    private static PIDFFConfig config = new PIDFFConfig(1, 1, 1);
    protected static Controller controller = new Controller(config, Controller.Type.POSITION);

    protected static NAR_TalonFX leader = new NAR_TalonFX(CLIMBER_LEADER_ID);

    public PWM winchServo = new PWM(WINCH_SERVO_ID);
    public PWM lockServo = new PWM(LOCK_SERVO_ID);

    public WinchMechanism() {
        super(controller, leader);
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
    }

    @Override
    protected void configController() {
       controller.setInputRange(CLIMBER_POSITiON_MIN, CLIMBER_POSITION_MAX);
       controller.configureFeedback(leader);
       controller.setTolerance(CLIMBER_TOLERANCE);
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