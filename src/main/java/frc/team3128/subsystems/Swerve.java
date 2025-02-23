package frc.team3128.subsystems;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.core.swerve.SwerveBase;
import common.core.swerve.SwerveConversions;
import common.core.swerve.SwerveModule;
import common.core.swerve.SwerveModuleConfig;
import common.core.swerve.SwerveModuleConfig.SwerveEncoderConfig;
import common.core.swerve.SwerveModuleConfig.SwerveMotorConfig;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.CmdSysId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.FieldConstants.FieldStates.*;
import static frc.team3128.Constants.VisionConstants.*;

public class Swerve extends SwerveBase {

    private static Swerve instance;

    private Pigeon2 gyro;

    public Supplier<Double> yaw;

    public static final MotorConfig driveMotorConfig = new MotorConfig(SwerveConversions.rotationsToMeters(1, DRIVE_WHEEL_CIRCUMFERENCE, DRIVE_MOTOR_GEAR_RATIO), 60, DRIVE_MOTOR_CURRENT_LIMIT, DRIVE_MOTOR_INVERTED, Neutral.BRAKE, StatusFrames.VELOCITY);

    public static final MotorConfig angleMotorConfig = new MotorConfig(SwerveConversions.rotationsToDegrees(1, DRIVE_ANGLE_GEAR_RATIO), 1, DRIVE_ANGLE_CURRENT_LIMIT, DRIVE_ANGLE_INVERTED, Neutral.BRAKE, StatusFrames.POSITION);

    public static final PIDFFConfig drivePIDConfig = new PIDFFConfig(DRIVE_MOTOR_KP, DRIVE_MOTOR_KI, DRIVE_MOTOR_KD, DRIVE_MOTOR_KS, DRIVE_MOTOR_KV, DRIVE_MOTOR_KA);

    public static final PIDFFConfig anglePIDConfig = new PIDFFConfig(DRIVE_ANGLE_KP, DRIVE_ANGLE_KI, DRIVE_ANGLE_KD);

    private static final SwerveModuleConfig Mod0 = new SwerveModuleConfig(
        0, 
        new SwerveMotorConfig(new NAR_TalonFX(MOD0_DRIVE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(MOD0_ANGLE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(MOD0_CANCODER_ID, DRIVETRAIN_CANBUS_NAME), MOD0_CANCODER_OFFSET, ANGLE_CANCODER_INVERTED),
        MAX_DRIVE_SPEED);

    private static final SwerveModuleConfig Mod1 = new SwerveModuleConfig(
        1, 
        new SwerveMotorConfig(new NAR_TalonFX(MOD1_DRIVE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(MOD1_ANGLE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(MOD1_CANCODER_ID, DRIVETRAIN_CANBUS_NAME), MOD1_CANCODER_OFFSET, ANGLE_CANCODER_INVERTED),
        MAX_DRIVE_SPEED);
        
    private static final SwerveModuleConfig Mod2 = new SwerveModuleConfig(
        2, 
        new SwerveMotorConfig(new NAR_TalonFX(MOD2_DRIVE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(MOD2_ANGLE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(MOD2_CANCODER_ID, DRIVETRAIN_CANBUS_NAME), MOD2_CANCODER_OFFSET, ANGLE_CANCODER_INVERTED),
        MAX_DRIVE_SPEED);
        
    private static final SwerveModuleConfig Mod3 = new SwerveModuleConfig(
        3, 
        new SwerveMotorConfig(new NAR_TalonFX(MOD3_DRIVE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), driveMotorConfig, drivePIDConfig),
        new SwerveMotorConfig(new NAR_TalonFX(MOD3_ANGLE_MOTOR_ID, DRIVETRAIN_CANBUS_NAME), angleMotorConfig, anglePIDConfig),
        new SwerveEncoderConfig(new CANcoder(MOD3_CANCODER_ID, DRIVETRAIN_CANBUS_NAME), MOD3_CANCODER_OFFSET, ANGLE_CANCODER_INVERTED),
        MAX_DRIVE_SPEED);

    public static final Translation2d[] moduleOffsets = {
        new Translation2d(DRIVE_WHEEL_BASE / 2.0, DRIVE_TRACK_WIDTH / 2.0), // front left - 0
        new Translation2d(DRIVE_WHEEL_BASE / 2.0, -DRIVE_TRACK_WIDTH / 2.0), // front right - 1
        new Translation2d(-DRIVE_WHEEL_BASE / 2.0, DRIVE_TRACK_WIDTH / 2.0), // back left - 2
        new Translation2d(-DRIVE_WHEEL_BASE / 2.0, -DRIVE_TRACK_WIDTH / 2.0) // back right - 3
    };
    
    private static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(moduleOffsets); 

    // x * kP = dx/dt && (v_max)^2 = 2*a_max*x
    public static final Constraints translationConstraints = new Constraints(MAX_DRIVE_SPEED, MAX_DRIVE_ACCELERATION);
    public static final PIDFFConfig translationConfig = new PIDFFConfig(20);// used to be 5//2 * MAX_DRIVE_ACCELERATION / MAX_DRIVE_SPEED); //Conservative Kp estimate (2*a_max/v_max)
    public static final Controller translationController = new Controller(translationConfig, Controller.Type.POSITION); //Displacement error to output velocity
    public static final double translationTolerance = 0.02;

    public static final Constraints rotationConstraints = new Constraints(MAX_DRIVE_ANGULAR_VELOCITY, MAX_DRIVE_ANGULAR_ACCELERATION);
    public static final PIDFFConfig rotationConfig = new PIDFFConfig(5); //Conservative Kp estimate (2*a_max/v_max)
    public static final Controller rotationController = new Controller(rotationConfig, Controller.Type.POSITION); //Angular displacement error to output angular velocity
    public static final double rotationTolerance = Angle.ofRelativeUnits(2, Units.Degree).in(Units.Radian);

    private static double translationPlateauThreshold = 5;
    private static double translationPlateauCount = 0;

    static {
        translationController.setTolerance(translationTolerance);
        translationController.setConstraints(translationConstraints);
        translationController.setDisableAtSetpoint(true);
        translationController.setSetpoint(0);
        
        rotationController.setTolerance(rotationTolerance);
        rotationController.setConstraints(rotationConstraints);
        rotationController.setDisableAtSetpoint(true);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setSetpoint(0);
    }

    private static Translation2d translationSetpoint = new Translation2d();
    private static Supplier<Rotation2d> rotationSetpointSupplier = ()-> new Rotation2d();

    public static boolean autoEnabled = false;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(swerveKinematics, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD, Mod0, Mod1, Mod2, Mod3);
        chassisVelocityCorrection = false;
        Timer.delay(1);
        gyro = new Pigeon2(PIDGEON_ID, DRIVETRAIN_CANBUS_NAME);
        Timer.delay(1);
        var x = gyro.getYaw();
        x.setUpdateFrequency(100);
        yaw = () -> gyro.getYaw().asSupplier().get().in(Units.Degree);

        gyro.optimizeBusUtilization();

        // initShuffleboard();
        initStateCheck();
    }

    @Override
    public double getYaw() {
        return yaw.get();
    }

    @Override
    public double getPitch() {
        return 0;
    }

    @Override
    public double getRoll() {
        return 0;
    }

    /**
     * @param velocity Desired robot velocity ROBOT RELATIVE
     */
    @Override
    public void drive(ChassisSpeeds velocity){
        if(Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond) < TRANSLATIONAL_DEADBAND && translationController.isEnabled()) {
            // if (!rotationController.isEnabled()) {
                Translation2d error = getDisplacementTo(translationSetpoint);
                Translation2d output = new Translation2d(translationController.calculate(error.getNorm()), error.getAngle());
                velocity.vxMetersPerSecond = output.getX();
                velocity.vyMetersPerSecond = output.getY();
            // }
        }
        // else translationController.disable();

        if(Math.abs(velocity.omegaRadiansPerSecond) < ROTATIONAL_DEADBAND && rotationController.isEnabled()) {
            Rotation2d error = getAngularDisplacementTo(rotationSetpointSupplier.get());
            velocity.omegaRadiansPerSecond = rotationController.calculate(error.getRadians()); 
        }
        // else rotationController.disable();

        assign(velocity);
        if(translationController.isEnabled() && atTranslationSetpoint()){
            translationController.disable();
            rotationController.enable();
        }
        if(rotationController.isEnabled() && atRotationSetpoint()) rotationController.disable();
    }

    public Command getDriveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta){
        return new FunctionalCommand(
            ()-> {}, 
            ()-> drive(inputToChassisSpeeds(x, y, theta)),
            (Boolean interrupted)-> stop(),
            ()-> false,
            this);
    }

    private ChassisSpeeds inputToChassisSpeeds(DoubleSupplier x, DoubleSupplier y, DoubleSupplier z){
        final Translation2d translation = adjustControllerInputs(x.getAsDouble(), y.getAsDouble(), true).times(MAX_DRIVE_SPEED);
        final double rotation = -Math.copySign(Math.pow(z.getAsDouble(), 3/2), z.getAsDouble()) * MAX_DRIVE_ANGULAR_VELOCITY;
        return new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    public void setPose(Pose2d pose){
        rotateTo(pose.getRotation());
        moveTo(pose.getTranslation());
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        resetGyro(pose.getRotation().getDegrees());
        odometry.resetPosition(getGyroRotation2d(), getPositions(), pose);
    }

    public void moveTo(Translation2d translation) {
        translationSetpoint = translation;
        translationController.enable();
    }

    public void moveBy(Translation2d translation) {
        translationSetpoint = getPose().getTranslation().plus(translation);
        translationController.enable();
    }

    public void rotateTo(Rotation2d theta) {
        rotationSetpointSupplier = ()->theta;
        rotationController.enable();
    }
    
    public void rotateTo(Translation2d translation) {
        rotationSetpointSupplier = ()-> new Rotation2d(translation.getX(), translation.getY());
        rotationController.enable();
    }

    public void rotateBy(Rotation2d dTheta) {
        Rotation2d curGyroRotation = getGyroRotation2d();
        rotationSetpointSupplier = ()-> curGyroRotation.plus(dTheta);
        rotationController.enable();
    }

    public boolean atRotationSetpoint() {
        return Math.abs(getAngleTo(rotationSetpointSupplier.get())) < rotationTolerance;
    }

    public boolean atTranslationSetpoint() {
        if(getDistanceTo(translationSetpoint) < translationTolerance) translationPlateauCount++;
        if(translationPlateauCount >= translationPlateauThreshold) {
            translationPlateauCount = 0;
            return true;
        }
        return false;
    }

    public void snapToAngle() {
        final Rotation2d gyroAngle = Swerve.getInstance().getGyroRotation2d();
        Rotation2d setpoint = Collections.min(
                            snapToAngles,
                            Comparator.comparing(
                                (Rotation2d angle) -> Math.abs(gyroAngle.minus(angle).getDegrees()))
                            );
        rotateTo(setpoint);
    }

    public void snapToElement() {
        final Pose2d setpoint = getPose().nearest(allianceFlip(reefPoses.appendAll(sourcePoses).asJava()));
        rotateTo(setpoint.getRotation());
    }

    public void pathToReef(Pose2d setpoint) {
        rotateTo(setpoint.getRotation());
        moveTo(setpoint.getTranslation());
    }

    public void pathToReef(boolean isRight) {
        final List<Pose2d> setpoints;
        setpoints = isRight ? reefRight.asJava() : reefLeft.asJava();
        final Pose2d setpoint = getPose().nearest(allianceFlip(setpoints));
        pathToReef(setpoint);
    }

    public void pathToSource() {
        Pose2d setpoint = getPose().nearest(allianceFlip(sourcePoses.asJava()));
        // Translation2d ram = new Translation2d(-0.05,0).rotateBy(setpoint.getRotation());
        rotateTo(setpoint.getRotation());
        moveTo(setpoint.getTranslation());
    }

    public boolean isConfigured() {
        for (final SwerveModule module : modules) {
            final double CANCoderAngle = module.getAbsoluteAngle().getDegrees();
            final double AngleMotorAngle = module.getAngleMotor().getPosition();
            NAR_Shuffleboard.addData("Modules Status", "Module " + module.moduleNumber + " Drive", ()-> module.getDriveMotor().getTemperature() != 0 , 0, module.moduleNumber);
            NAR_Shuffleboard.addData("Modules Status", "Module " + module.moduleNumber + " Angle", ()-> module.getAngleMotor().getTemperature() != 0 , 0, module.moduleNumber);
            NAR_Shuffleboard.addData("Modules Status", "Module " + module.moduleNumber + " CANCoder", ()-> module.getAngleEncoder().getVersion().getValue() != 0, 0, module.moduleNumber);
            if (CANCoderAngle == 0 || AngleMotorAngle == 0) return false;
        }
        return true;
    }

    public Pigeon2 getGyro() {
        return gyro;
    }

    @Override
    public void resetGyro(double reset) {
        gyro.setYaw(reset);
    }

    public Command characterize(double startDelay, double rampRate, double targetPosition) {
        NAR_Motor driveMotor = modules[0].getDriveMotor();
        final double startPos = driveMotor.getPosition();
        return new CmdSysId(
            getName(), 
            (volts)-> setDriveVoltage(volts), 
            ()-> driveMotor.getVelocity(), 
            ()-> driveMotor.getPosition(), 
            startDelay, 
            rampRate, 
            startPos + targetPosition, 
            true, 
            this
        );
    }

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData("Swerve", "Throttle", ()-> this.throttle, 4, 3);

        NAR_Shuffleboard.addData("Auto", "Reef", ()-> getPose().nearest(allianceFlip(reefPoses.asJava())).toString(), 3, 3);

        NAR_Shuffleboard.addData("Auto", "Translation Enabled", ()-> translationController.isEnabled(), 0, 0);
        NAR_Shuffleboard.addData("Auto", "At Setpoint", ()-> atTranslationSetpoint(), 0, 1);
        NAR_Shuffleboard.addData("Auto", "Error", ()-> getDistanceTo(translationSetpoint), 1, 0);

        // NAR_Shuffleboard.addData("Auto", "Rotation Enabled", ()-> rotationController.isEnabled(), 0, 2);
        // NAR_Shuffleboard.addData("Auto", "At Setpoint", ()-> atRotationSetpoint(), 0, 3);
        // NAR_Shuffleboard.addData("Auto", "Error", ()-> getAngleTo(rotationSetpointSupplier.get()), 1, 3);
    }

    public static void disable() {
        translationController.disable();
        rotationController.disable();
        getInstance().stop();
    }
}
