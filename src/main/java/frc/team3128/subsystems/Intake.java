package frc.team3128.subsystems;
import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.IntakeConstants.*;

public class Intake extends VoltageSubsystemBase {

    private static Intake instance;

    public static NAR_CANSpark rollerMotor = new NAR_CANSpark(ROLLER_ID, ControllerType.CAN_SPARK_FLEX);
    
        private Intake() {
            super(CURRENT_THRESHOLD, rollerMotor);
    }

    @Override
    public void initShuffleboard() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initShuffleboard'");
    }

    @Override
    protected void configMotors() {
        MotorConfig intakeConfig = new MotorConfig(
            ROLLER_GEAR_RATIO,
            ROLLER_SAMPLE_PER_MINUTE,
            ROLLER_STATOR_CURRENT_LIMIT,
            ROLLER_INVERT,
            ROLLER_NEUTRAL_MODE,
            ROLLER_STATUS_FRAME
        );

        rollerMotor.configMotor(intakeConfig);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }
}
