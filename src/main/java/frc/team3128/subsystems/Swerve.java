package frc.team3128.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;
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
import common.hardware.camera.Camera;
import common.hardware.motorcontroller.NAR_Motor;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.CmdSysId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.FieldConstants.FieldStates.*;
import static frc.team3128.Constants.VisionConstants.*;
import static frc.team3128.Constants.DriveConstants.*;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.Robot;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;
import frc.team3128.Robot;
import frc.team3128.RobotContainer;

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
    public static final PIDFFConfig translationConfig = new PIDFFConfig(3.5, 0, 0);//3 // used to be 5//2 * MAX_DRIVE_ACCELERATION / MAX_DRIVE_SPEED); //Conservative Kp estimate (2*a_max/v_max)
    public static final Controller translationController = new Controller(translationConfig, Controller.Type.POSITION); //Displacement error to output velocity
    public static final double translationTolerance = 0.03;

    public static DoubleSupplier kPSupplier, kISupplier, kDSupplier;


    public static final double elevatorStartDist = 0.4;

    public static final Constraints rotationConstraints = new Constraints(MAX_DRIVE_ANGULAR_VELOCITY, MAX_DRIVE_ANGULAR_ACCELERATION);
    public static final PIDFFConfig rotationConfig = new PIDFFConfig(10); //Conservative Kp estimate (2*a_max/v_max)
    public static final Controller rotationController = new Controller(rotationConfig, Controller.Type.POSITION); //Angular displacement error to output angular velocity
    public static final double rotationTolerance = Angle.ofRelativeUnits(1, Units.Degree).in(Units.Radian);

    private static double translationPlateauThreshold = 40;
    private static double translationPlateauCount = 0;


    private static double elevatorPlateauThreshold = 10; //~half second
    private static double elevatorPlateauCount = 0;

    private static double rotationPlateauThreshold = 10;
    private static double rotationPlateauCount = 0;

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

    public static boolean autoMoveEnabled = false;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(swerveKinematics, SVR_STATE_STD, SVR_VISION_MEASUREMENT_STD, Mod0, Mod1, Mod2, Mod3);
        chassisVelocityCorrection = true;
        Timer.delay(1);
        gyro = new Pigeon2(PIDGEON_ID, DRIVETRAIN_CANBUS_NAME);
        Timer.delay(1);
        var x = gyro.getYaw();
        x.setUpdateFrequency(100);
        yaw = () -> gyro.getYaw().asSupplier().get().in(Units.Degree);

        gyro.optimizeBusUtilization();

        initShuffleboard();
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
        // if ((Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond) >= TRANSLATIONAL_DEADBAND) ||
        if(Math.abs(velocity.omegaRadiansPerSecond) >= ROTATIONAL_DEADBAND) {
            translationController.disable();
            rotationController.disable();
        }

        if(Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond) < TRANSLATIONAL_DEADBAND && translationController.isEnabled()) {
                Translation2d error = getTranslation2dTo(translationSetpoint);
                Translation2d output = new Translation2d(translationController.calculate(error.getNorm()), error.getAngle());
                velocity.vxMetersPerSecond = output.getX();
                velocity.vyMetersPerSecond = output.getY();
        }

        if(Math.abs(velocity.omegaRadiansPerSecond) < ROTATIONAL_DEADBAND && rotationController.isEnabled()) {
            Rotation2d error = getRotation2dTo(rotationSetpointSupplier.get());
            velocity.omegaRadiansPerSecond = rotationController.calculate(error.getRadians()); 
        }

        assign(velocity);
        if(translationController.isEnabled() && atTranslationSetpoint()) translationController.disable();
        if(rotationController.isEnabled() && atRotationSetpoint() && !translationController.isEnabled()) rotationController.disable();
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

    public void resetOdometryNoGyro(Pose2d pose) {
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
        if(Math.abs(getAngleTo(rotationSetpointSupplier.get())) < rotationTolerance) rotationPlateauCount++;
        else {
            rotationPlateauCount = 0;
            return false;
        }
        if (rotationPlateauCount >= rotationPlateauThreshold) {
            rotationPlateauCount = 0;
            return true;
        }
        return false;
    }

    public boolean atTranslationSetpoint() {
        if(getDistanceTo(translationSetpoint) < translationTolerance) translationPlateauCount++;
        else {
            translationPlateauCount = 0;
            return false;
        }
        if(translationPlateauCount >= translationPlateauThreshold) {
            return true;
        }
        return false;
    }

    public boolean atElevatorDist() {
        if(getDistanceTo(translationSetpoint) < elevatorStartDist) elevatorPlateauCount++;
        else {
            elevatorPlateauCount = 0;
            return false;
        }
        if(elevatorPlateauCount >= elevatorPlateauThreshold) {
            elevatorPlateauCount = 0;
            return true;
        }
        return false;
    }

    public void snapToAngle() {
        final Rotation2d gyroAngle = Swerve.getInstance().getGyroRotation2d();
        Rotation2d setpoint = Collections.min(
                            DriveConstants.snapToAngles,
                            Comparator.comparing(
                                (Rotation2d angle) -> Math.abs(gyroAngle.minus(angle).getDegrees()))
                            );
        rotateTo(setpoint);
    }

    public void snapToElement() {
        final Pose2d setpoint = nearestPose2d(allianceFlip(reefPoses.appendAll(sourcePoses).asJava()));
        rotateTo(setpoint.getRotation());
    }

    public Pose2d getClosestReef() {
        return nearestPose2d(allianceFlip(reefPoses.asJava()));
    }

    public Command autoAlign(FieldStates state) {
        return autoAlign(state.getPose2d());
    }

    // public Command autoAlign(boolean isRight, BooleanSupplier shouldRam) {
    //     final List<Pose2d> setpoints = isRight ? reefRight.asJava() : reefLeft.asJava();
    //     final List<Pose2d> fudgelessSetpoints = isRight ? fudgelessReefRight.asJava() : fudgelessReefLeft.asJava();
    //     Supplier<Pose2d> pose = () -> (shouldRam.getAsBoolean() ? nearestPose2d(allianceFlip(setpoints)) : nearestPose2d(allianceFlip(fudgelessSetpoints)));
    //     return navigateTo(pose);
    // }

    // public Command autoAlignSource() {
    //     return navigateTo(()-> nearestPose2d(allianceFlip(List.of(SOURCE_1.getPose2d(), SOURCE_2.getPose2d()))));
    // }

    public Command autoAlign(Pose2d pose) {
        final Pose2d flippedPose = allianceFlipRotationally(pose);
        return autoAlign(() -> flippedPose, ()->true);
    }

    public Command autoAlignBargeSimple() {
        return autoAlign(() -> allianceFlip(new Pose2d(new Translation2d(7.7, getPose().getY()), Rotation2d.fromDegrees(0))), () -> false);
    }

    public static boolean toggleBargeScore = true;
    public Command runBargeScore() {
        return either(
            sequence(
                Commands.runOnce(() -> toggleBargeScore = !toggleBargeScore),
                RobotManager.getInstance().setStateCommand(RobotStates.RPB),
                autoAlignBargeSimple()
            ), 
            sequence(
                Commands.runOnce(() -> toggleBargeScore = !toggleBargeScore),
                RobotManager.getInstance().setStateCommand(RobotStates.RSB),
                waitSeconds(0.5),
                RobotManager.getInstance().setStateCommand(RobotStates.NEUTRAL)
            ), 
            () -> toggleBargeScore
        );
        // if(toggleBargeScore){
        //     return sequence(
        //         autoAlignBargeSimple()
        //     ).beforeStarting(RobotManager.getInstance().setStateCommand(RobotStates.RPB));
        // }
        // toggleBargeScore = !toggleBargeScore;

        // return sequence(RobotManager.getInstance().setStateCommand(RobotStates.RSB),waitSeconds(0.5),RobotManager.getInstance().setStateCommand(RobotStates.NEUTRAL));
    }

    public Command autoAlign(Supplier<Pose2d> pose, BooleanSupplier shouldRam) {
        return Commands.sequence(
            navigateTo(pose),
            ram(pose).onlyIf(shouldRam)
        );
    }

    public Command navigateTo(Supplier<Pose2d> pose) {
        return navigateTo(pose, 2);
    }

    public Command navigateTo(Supplier<Pose2d> pose, double timeout) {
        return Commands.startEnd(
            ()-> {
                setThrottle(1); //force throttle to max speed
                Swerve.autoMoveEnabled = true;
                setPose(pose.get()); //set setpoints and enable controllers
            },
            ()-> {
                disable();
                Swerve.autoMoveEnabled = false;
            }
        ).until(()-> atTranslationSetpoint())
         .withTimeout(timeout);
    }

    public Command ram(Supplier<Pose2d> pose) {
        return ram(pose, 1);
    }

    public Command ram(Supplier<Pose2d> pose, double timeout) {
        return Commands.startEnd(
            ()-> {
                setThrottle(1);
                moveBy(RAM_FACTOR.rotateBy(pose.get().getRotation()));
            },
            ()-> {
                disable();
            }
        ).until(()-> atTranslationSetpoint())
         .withTimeout(timeout);
    }

    public boolean isConfigured() {
        for (final SwerveModule module : modules) {
            final double CANCoderAngle = module.getAbsoluteAngle().getDegrees();
            final double AngleMotorAngle = module.getAngleMotor().getPosition();
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

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData("Swerve", "Throttle", this::getThrottle, 4, 3);


        NAR_Shuffleboard.addData("Auto", "Translation Enabled", ()-> translationController.isEnabled(), 0, 0);
        NAR_Shuffleboard.addData("Auto", "At Setpoint", ()-> atTranslationSetpoint(), 0, 1);
        NAR_Shuffleboard.addData("Auto", "Error", ()-> getDistanceTo(translationSetpoint), 1, 0);
        NAR_Shuffleboard.addData("Auto", "Count", ()-> translationPlateauCount, 1, 1);
        kPSupplier = NAR_Shuffleboard.debug("Auto", "kP", translationConfig.kP, 2, 0);
        kISupplier = NAR_Shuffleboard.debug("Auto", "kI", translationConfig.kI, 2, 1);
        kDSupplier = NAR_Shuffleboard.debug("Auto", "kD", translationConfig.kD, 2, 2);
    }

    public static void disable() {
        translationController.disable();
        rotationController.disable();
        getInstance().stop();
    }
}
