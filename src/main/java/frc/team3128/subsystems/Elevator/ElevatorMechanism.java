package frc.team3128.subsystems.Elevator;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.subsystems.PositionSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
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
    }   

    public SysIdRoutine driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.2).per(Second), Volts.of(4), null),
        new SysIdRoutine.Mechanism((v) -> runVolts(v.in(Volts)), this::logMotors, this)
    );

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        if (direction == SysIdRoutine.Direction.kForward) {
            return driveRoutine.quasistatic(direction).onlyWhile(() -> left.getPosition() < 0.8 * ELEVATOR_POSITION_MAX);
        } else {
            return driveRoutine.quasistatic(direction).onlyWhile(() -> left.getPosition() > 0.2 * ELEVATOR_POSITION_MAX);
        }
    }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        if (direction == SysIdRoutine.Direction.kForward) {
            return driveRoutine.dynamic(direction).onlyWhile(() -> left.getPosition() < 0.8 * ELEVATOR_POSITION_MAX);
        } else {
            return driveRoutine.dynamic(direction).onlyWhile(() -> left.getPosition() > 0.2 * ELEVATOR_POSITION_MAX);
        }
    }
    
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private final MutDistance position = Meters.mutable(0);
    private final MutLinearVelocity velocity = MetersPerSecond.mutable(0);
    public void logMotors(SysIdRoutineLog log){
        // log.motor("position").linearPosition(Meters.of(left.getPosition()));
        // log.motor("velocity").linearVelocity(MetersPerSecond.of(left.getVelocity() / 60.0));
        // log.motor("voltage").voltage(Volts.of(12 * left.getAppliedOutput()));
        log.motor("elevator-motor")
            .linearPosition(position.mut_replace(left.getPosition(), Meters))
            .linearVelocity(velocity.mut_replace(left.getVelocity(), MetersPerSecond))
            .voltage(appliedVoltage.mut_replace(left.getAppliedOutput() * 12, Volts));
    }
}