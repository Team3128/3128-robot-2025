package frc.team3128;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team3128.subsystems.Swerve;



public class Constants {

    public static class RobotConstants {

        public static final double maxVelocity = 0;
        public static final double maxAcceleration = 0;

        public static final double maxAngularVelocity = 0;
        public static final double maxAngularAcceleration = 0;

        public static final double slow = 0.3;
        public static final double medium = 0.6;
        public static final double fast = 1;

        // Distance between front and rear wheels
        public static final double wheelBase = 0;
        // Distance between the left and right wheels
        public static final double trackWidth = 0;

        public static final double robotLength = 0;
        public static final double robotWidth = 0;
        public static final double bumperLength = 0;
        public static final double bumperWidth = 0;

        public static final double wheelDiameter = 0;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double neutralHeight = 0;
    }

    public static class AutoConstants {

        // public static final var autoConstraints = null;

        /* Translation PID Values */
        public static final double translationKP = DriveConstants.translationConfig.kP;
        public static final double translationKI = DriveConstants.translationConfig.kI;
        public static final double translationKD = DriveConstants.translationConfig.kD;
      
        /* Rotation PID Values */
        public static final double rotationKP = DriveConstants.rotationConfig.kP;
        public static final double rotationKI = DriveConstants.rotationConfig.kI;
        public static final double rotationKD = DriveConstants.rotationConfig.kD;

    }

    public static class DriveConstants {

        public static final double controllerPOVOffset = -90;

        public static final double driveMotorGearRatio = 0;
        public static final double angleMotorGearRatio = 0; 

        // public static final var kinematics = null; 

        /* Motor Current Limiting */
        public static final int angleMotorCurrentLimit = 0;
        public static final int driveMotorCurrentLimit = 0;
        public static final int motorStallCurrentLimit = 0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0;
        public static final double angleKI = 0;
        public static final double angleKD = 0;
        public static final double angleKF = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveKF = 0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0;
        public static final double driveKV = 0;
        public static final double driveKA = 0;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final MotorConfig driveMotorConfig = null;

        public static final MotorConfig angleMotorConfig = null;

        public static final PIDFFConfig drivePIDConfig = new PIDFFConfig(driveKP, driveKI, driveKD, driveKS, driveKV, driveKA);

        public static final PIDFFConfig anglePIDConfig = new PIDFFConfig(angleKP, angleKI, angleKD);

        public static final Constraints translationConstraints = new Constraints(RobotConstants.maxVelocity, RobotConstants.maxAcceleration);
        public static final PIDFFConfig translationConfig = new PIDFFConfig(0, 0, 0, 0, 0, 0);
        public static final Controller translationController = new Controller(translationConfig, Controller.Type.POSITION);
        public static final double TRANSLATION_TOLERANCE = 0;

        public static final Constraints rotationConstraints = new Constraints(Units.radiansToDegrees(RobotConstants.maxAngularAcceleration), Units.radiansToDegrees(RobotConstants.maxAngularAcceleration));
        public static final PIDFFConfig rotationConfig = new PIDFFConfig(0, 0, 0, 0, 0, 0);
        public static final Controller rotationController = new Controller(rotationConfig, Controller.Type.POSITION);
        public static final double TURN_TOLERANCE = 0;

        static {
            translationController.setTolerance(TRANSLATION_TOLERANCE);
            translationController.setOutputRange(-RobotConstants.maxVelocity, RobotConstants.maxVelocity);
            translationController.setDisableAtSetpoint(true);
            
            rotationController.setTolerance(TURN_TOLERANCE);
            rotationController.setOutputRange(-RobotConstants.maxAngularVelocity, RobotConstants.maxAngularVelocity);
            rotationController.setDisableAtSetpoint(true);
            rotationController.enableContinuousInput(-180, 180);
        }

        public static final List<Rotation2d> snapToAngles = new ArrayList<>();
        static {
            snapToAngles.add(Rotation2d.fromDegrees(-180));
            snapToAngles.add(Rotation2d.fromDegrees(-90));
            snapToAngles.add(Rotation2d.fromDegrees(0));
            snapToAngles.add(Rotation2d.fromDegrees(90));
            snapToAngles.add(Rotation2d.fromDegrees(180));
        }
    }

    public static class SwerveConstants {
        /* Module Device IDs */
        public static final int MOD0_DRIVE_MOTOR_ID = 1;
        public static final int MOD0_ANGLE_MOTOR_ID = 2;
        public static final int MOD0_CANCODER_ID = 10;
        public static final int MOD1_DRIVE_MOTOR_ID = 3;
        public static final int MOD1_ANGLE_MOTOR_ID = 4;
        public static final int MOD1_CANCODER_ID = 11;
        public static final int MOD2_DRIVE_MOTOR_ID = 5;
        public static final int MOD2_ANGLE_MOTOR_ID = 6;
        public static final int MOD2_CANCODER_ID = 12;
        public static final int MOD3_DRIVE_MOTOR_ID = 7;
        public static final int MOD3_ANGLE_MOTOR_ID = 8;
        public static final int MOD3_CANCODER_ID = 13;

        /* Cancoder Offsets */
        public static final double MOD0_CANCODER_OFFSET = 19.16015624999 - 180;
        public static final double MOD1_CANCODER_OFFSET = -113.818359375;
        public static final double MOD2_CANCODER_OFFSET = -67.412109375 + 180;
        public static final double MOD3_CANCODER_OFFSET = 62.13867187499;

        public static final double RAMP_TIME = 3;

        /* Drivetrain Constants */
        public static final double ROBOT_MASS = 35.210; //kg
        public static final double ROBOT_MOI = 3.385; //kg m^2 mass * (trackWidth / 2) * (Ka angular / Ka linear)
        public static final double WHEEL_COF = 1.43;
        public static final double DRIVE_BUMPER_LENGTH = Units.inchesToMeters(5);
        public static final double DRIVE_TRACK_WIDTH = Units.inchesToMeters(20.75); //Hand measure later
        public static final double DRIVE_WHEEL_BASE = Units.inchesToMeters(20.75); //Hand measure later
        public static final double ROBOT_LENGTH = Units.inchesToMeters(26.5) + DRIVE_BUMPER_LENGTH; // bumperLength + trackWidth;
        public static final double DRIVE_WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double DRIVE_MOTOR_GEAR_RATIO = 225.0 / 42.0;
        public static final double DRIVE_ANGLE_GEAR_RATIO = (300.0 / 13.0); 

        /* Swerve Current Limiting */
        public static final int DRIVE_ANGLE_CURRENT_LIMIT = 30; //30
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60; //40;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double DRIVE_ANGLE_KP = 0.15 * 30; // 0.6; // citrus: 0.3 //0.15
        public static final double DRIVE_ANGLE_KI = 0.0;
        public static final double DRIVE_ANGLE_KD = 0.0; // 12.0; // citrus: 0
        public static final double DRIVE_ANGLE_KF = 0.0;

        /* Drive Motor PID Values */
        public static final double DRIVE_MOTOR_KP = 4e-5; //4e-5, //0.05
        public static final double DRIVE_MOTOR_KI = 0.0;
        public static final double DRIVE_MOTOR_KD = 0.0;
        public static final double DRIVE_MOTOR_KF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double DRIVE_MOTOR_KS = 0.19057;//0.60094; // 0.19225;
        public static final double DRIVE_MOTOR_KV = 2.01208;//1.1559;  // 2.4366
        public static final double DRIVE_MOTOR_KA = 0.09043; //0.12348; // 0.34415

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double MAX_DRIVE_SPEED = 4.57;//4.8; //meters per second - 16.3 ft/sec
        public static final double MAX_ATTAINABLE_DRIVE_SPEED = MAX_DRIVE_SPEED; //Stole from citrus.
        public static final double MAX_DRIVE_ACCELERATION = 5;
        public static final double MAX_DRIVE_ANGULAR_VELOCITY = 8;
        public static final double MAX_DRIVE_ANGULAR_ACCELERATION = 2 * Math.PI; //I stole from citrus.

        /* Motor and Sensor IDs */
        public static final int PIDGEON_ID = 9; 
        public static final String DRIVETRAIN_CANBUS_NAME = "drivetrain";
        public static final double TRANSLATIONAL_DEADBAND = 0.5;
        public static final double ROTATIONAL_DEADBAND = 0.1;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean DRIVE_ANGLE_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean ANGLE_CANCODER_INVERTED = false;

        public static final double DRIVE_TURN_KP = 5;
        public static final double DRIVE_TURN_KI = 0;
        public static final double DRIVE_TURN_KD = 0;
        public static final double DRIVE_TURN_KS = 0.1; //0.05748
        public static final double DRIVE_TURN_KV = 0.01723; //0.01723
        public static final double DRIVE_TURN_KA = 0.0064; //0.0064
        public static final double DRIVE_TURN_KG = 0;

        public static final List<Rotation2d> snapToAngles = new ArrayList<>();
        static {
            snapToAngles.add(Rotation2d.fromDegrees(-180));
            snapToAngles.add(Rotation2d.fromDegrees(-90));
            snapToAngles.add(Rotation2d.fromDegrees(0));
            snapToAngles.add(Rotation2d.fromDegrees(90));
            snapToAngles.add(Rotation2d.fromDegrees(180));
        }
    }

    public static class VisionConstants {

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1, 0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(5));

        public static final HashMap<Integer, Pose2d> APRIL_TAG_POS = new HashMap<Integer,Pose2d>();
    }
    
    public static class FieldConstants{

        public static final double FIELD_X_LENGTH = Units.inchesToMeters(651.25); // meters
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(315.5); // meters

        public enum FieldStates {
            BLUE_BARGE,
            CENTER_BARGE,
            RED_BARGE,
            REEF_1,
            REEF_2,
            REEF_3,
            REEF_4,
            REEF_5,
            REEF_6,
            PIECE_1,
            PIECE_2,
            PIECE_3,
            CORAL_1,
            CORAL_2,
            PROCESSOR
        }


        public static Pose2d allianceFlip(Pose2d pose) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flip(pose);
            }
            return pose;
        } 

        public static Translation2d allianceFlip(Translation2d translation) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flipTranslation(translation);
            }
            return translation;
        }

        public static Rotation2d allianceFlip(Rotation2d rotation) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flipRotation(rotation);
            }
            return rotation;
        }

        public static Pose2d flip(Pose2d pose) {
            return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
        }

        public static Translation2d flipTranslation(Translation2d translation) {
            return new Translation2d (
                FIELD_X_LENGTH - translation.getX(),
                translation.getY()
            );
        }

        public static Rotation2d flipRotation(Rotation2d rotation) {
            return Rotation2d.fromDegrees(MathUtil.inputModulus(180 - rotation.getDegrees(), -180, 180));
        }

        public static Rotation2d flipRotation(double rotation) {
            return Rotation2d.fromDegrees(MathUtil.inputModulus(180 - rotation, -180, 180));
        }

        public static Translation2d adjustControllerInputs(double x, double y, boolean fieldRelative) {
            return adjustControllerInputs(new Translation2d(x, y), fieldRelative);
        }

        public static Translation2d adjustControllerInputs(Translation2d translation, boolean fieldRelative) {
            Rotation2d rotation = Rotation2d.fromDegrees(-90);
            if(Robot.getAlliance() == Alliance.Red || !fieldRelative) {
                System.out.println("Alliance Red Detected");
                rotation = Rotation2d.fromDegrees(90);
            }
            return translation.rotateBy(rotation);
        }
    }

    public static class LedConstants{

        public static final int WHITE_VALUE = 0; //leds used don't have a white value
        public static final double r_SPEED = 0.75;
        public static final double c_SPEED = 1;
        public static final int STARTING_ID = 8;
        public static final int PIVOT_COUNT = 200;
        public static final int PIVOT_FRONT = 40;
        public static final int PIVOT_BACK = 50;
        public static final int NUM_LED = PIVOT_FRONT - 10;
        public static final int SPARKING = 1;
        public static final double COOLING = 0.3;
        public static final double HOLDING_SPEED = 2;
        public static final double BRIGHTNESS = 1;
        public static final int OFFSET = 5 + 55;

        public static class RainbowAnimation {
            public static final double BRIGHTNESS = 1;
            public static final double SPEED = 1;

        }

        public enum Colors {
            OFF(0,0,0,false),
            ERROR(255, 0, 0, false),
            PIECE(0, 255, 0, false),
            CONFIGURED(0,255,0,false),
            BLUE(48, 122, 171, false),
            RED(171, 48, 97, false),
            PURPLE(255, 0, 255, false),
            GREEN(0, 255, 0, false),
            ORANGE(255, 50, 0, false),
    
            FLAME(0,0,0,true),
            CHARGE(255, 0, 0, true),
            DISCHARGE(0, 0, 0, true);
    
            public final int r;
            public final int b;
            public final int g;
            public final boolean animation;
    
            Colors(int r, int g, int b,boolean animation) {
                this.r = r;
                this.g = g;
                this.b = b;
                this.animation = animation;
            }
    
        }
    }
}