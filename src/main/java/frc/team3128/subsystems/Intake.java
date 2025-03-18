package frc.team3128.subsystems;
import common.core.subsystems.VoltageSubsystemBase;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;

import common.hardware.motorcontroller.NAR_CANSpark.ControllerType;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import static frc.team3128.Constants.IntakeConstants.*;

public class Intake extends VoltageSubsystemBase {

    private static Intake instance;

    public static NAR_TalonFX intakeMotor = new NAR_TalonFX(ROLLER_ID, "rio");
    public static NAR_CANSpark rollerMotor = new NAR_CANSpark(INTAKE_ID, ControllerType.CAN_SPARK_FLEX);
    public static NAR_TalonFX manipMotor = new NAR_TalonFX(MANIP_ID, "rio");
    //public static NAR_CANSpark serial = new NAR_CANSpark(SERIAL_ID, ControllerType.CAN_SPARK_FLEX);
    
    private Intake() {
        super(CURRENT_THRESHOLD, intakeMotor,manipMotor);

        configMotors();
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

        manipMotor.configMotor(intakeConfig);

        MotorConfig serialConfig = new MotorConfig(
            SERIAL_GEAR_RATIO,
            SERIAL_SAMPLE_PER_MINUTE,
            SERIAL_STATOR_CURRENT_LIMIT,
            SERIAL_INVERT,
            SERIAL_NEUTRAL_MODE,
            SERIAL_STATUS_FRAME
        );

        // serial.configMotor(serialConfig);
        
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }
}
