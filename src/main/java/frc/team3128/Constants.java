package frc.team3128;

import static edu.wpi.first.units.Units.Rotation;
import static frc.team3128.Constants.VisionConstants.APRIL_TAGS;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import com.pathplanner.lib.util.FlippingUtil;

import common.core.controllers.Controller;
import common.core.controllers.PIDFFConfig;
import common.hardware.motorcontroller.NAR_Motor.MotorConfig;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.hardware.motorcontroller.NAR_Motor.StatusFrames;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.team3128.subsystems.Swerve;



public class Constants {

    public static class DriveConstants {

        public static final double controllerPOVOffset = -90;

        public static final double slow = 0.3;
        public static final double medium = 0.6;
        public static final double fast = 1;

        /* Swerve Profiling Values */
        // Theoretical: v = 4.96824, omega = 11.5
        // Real: v = 4.5, omega = 10
        // For safety, use less than theoretical and real values
        public static final double MAX_DRIVE_SPEED = 4.5;//4.8; //meters per second - 16.3 ft/sec
        public static final double MAX_ATTAINABLE_DRIVE_SPEED = MAX_DRIVE_SPEED; //Stole from citrus.
        public static final double MAX_DRIVE_ACCELERATION = 3.4;//5;
        public static final double MAX_DRIVE_ANGULAR_VELOCITY = 2 * Math.PI;//10
        public static final double MAX_DRIVE_ANGULAR_ACCELERATION = 10;//2 * Math.PI; //I stole from citrus.

        public static final double driveMotorGearRatio = 0;
        public static final double angleMotorGearRatio = 150 / 7; 

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
        public static final double MOD0_CANCODER_OFFSET = -119.267578125;//-116.015625
        public static final double MOD1_CANCODER_OFFSET = -68.115234375;//-67.58789
        public static final double MOD2_CANCODER_OFFSET = 66.884765625;//65.0390625
        public static final double MOD3_CANCODER_OFFSET = 19.072265625;//19.24805


        public static final double RAMP_TIME = 3;

        /* Drivetrain Constants */
        public static final double ROBOT_MASS = 62; //kg
        public static final double WHEEL_COF = 1.2;
        public static final double DRIVE_BUMPER_LENGTH = Units.inchesToMeters(5);
        public static final double DRIVE_TRACK_WIDTH = Units.inchesToMeters(20.75); //Hand measure later
        public static final double DRIVE_WHEEL_BASE = Units.inchesToMeters(20.75); //Hand measure later
        public static final double ROBOT_LENGTH = Units.inchesToMeters(26.5) + DRIVE_BUMPER_LENGTH; // bumperLength + trackWidth;
        public static final double DRIVE_WHEEL_DIAMETER = 0.0486 * 2;
        public static final double DRIVE_WHEEL_CIRCUMFERENCE = (DRIVE_WHEEL_DIAMETER * Math.PI);
        public static final double ROBOT_MOI = ROBOT_MASS * (DRIVE_TRACK_WIDTH / 2) * 0.44965 / 0.4443; //kg m^2 mass * (trackWidth / 2) * (Ka angular / Ka linear)

        public static final double closedLoopRamp = 0.0;

        public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
        public static final double DRIVE_ANGLE_GEAR_RATIO = (150.0 / 7.0); // 300.0 / 13.0

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
        public static final double DRIVE_MOTOR_KS = 0.16746;//0.13023; //0.19057;//0.60094; // 0.19225;
        public static final double DRIVE_MOTOR_KV = 1.95619;//1.92348; //2.01208;//1.1559;  // 2.4366
        public static final double DRIVE_MOTOR_KA = 0.4443;//0.10274; //0.09043; //0.12348; // 0.34415

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

        public static final List<Rotation2d> snapToAngles = List.of(
            Rotation2d.fromDegrees(-180),
            Rotation2d.fromDegrees(-120),
            Rotation2d.fromDegrees(-60),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(60),
            Rotation2d.fromDegrees(120),
            Rotation2d.fromDegrees(180)
        );
    }

    public static class VisionConstants {

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.1, 0.1,Units.degreesToRadians(3));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(5));

        public static final List<AprilTag> APRIL_TAGS = Arrays.asList(
            new AprilTag(1, new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(126)))),
            new AprilTag(2, new Pose3d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(234)))),
            new AprilTag(3, new Pose3d(Units.inchesToMeters(455.15), Units.inchesToMeters(317.15), Units.inchesToMeters(51.25), new Rotation3d(0, Math.toRadians(0), Math.toRadians(270)))),
            new AprilTag(4, new Pose3d(Units.inchesToMeters(365.20), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(0)))),
            new AprilTag(5, new Pose3d(Units.inchesToMeters(265.20), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(0)))),
            new AprilTag(6, new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(300)))),
            new AprilTag(7, new Pose3d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))),
            new AprilTag(8, new Pose3d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(60)))),
            new AprilTag(9, new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(120)))),
            new AprilTag(10, new Pose3d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)))),
            new AprilTag(11, new Pose3d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(240)))),
            new AprilTag(12, new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(54)))),
            new AprilTag(13, new Pose3d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Units.inchesToMeters(58.50), new Rotation3d(0, Math.toRadians(0), Math.toRadians(306)))),
            new AprilTag(14, new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(241.64), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(180)))),
            new AprilTag(15, new Pose3d(Units.inchesToMeters(325.68), Units.inchesToMeters(75.39), Units.inchesToMeters(73.54), new Rotation3d(0, Math.toRadians(30), Math.toRadians(180)))),
            new AprilTag(16, new Pose3d(Units.inchesToMeters(235.73), Units.inchesToMeters(-0.15), Units.inchesToMeters(51.25), new Rotation3d(0, Math.toRadians(0), Math.toRadians(90)))),
            new AprilTag(17, new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(240)))),
            new AprilTag(18, new Pose3d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)))),
            new AprilTag(19, new Pose3d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(120)))),
            new AprilTag(20, new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(60)))),
            new AprilTag(21, new Pose3d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.58), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)))),
            new AprilTag(22, new Pose3d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Units.inchesToMeters(12.13), new Rotation3d(0, Math.toRadians(0), Math.toRadians(300))))
        );
    }
    
    public static class FieldConstants{

        public static final double FIELD_X_LENGTH = Units.inchesToMeters(690.875); // meters = 17.548
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(317); // meters = 8.052
        public static final Translation2d FIELD = new Translation2d(FIELD_X_LENGTH, FIELD_Y_LENGTH);
        public static final Translation2d CENTER_FIELD = FIELD.div(2);
        public static final Translation2d FUDGE_FACTOR = new Translation2d(0.1, 0);

        public static final Translation2d reefShift = new Translation2d(0.35/2, 0.);

        public enum FieldStates {
            A(18, false),
            B(18, true),
            C(17, false),
            D(17, true),
            E(22, false),
            F(22, true),
            G(21, false),
            H(21, true),
            I(20, false),
            J(20, true),
            K(19, false),
            L(19, true),

            SOURCE_1(new Pose2d(new Translation2d(1.267, 0.753), Rotation2d.fromDegrees(55))),
            SOURCE_2(new Pose2d(new Translation2d(1.267, FIELD_Y_LENGTH-0.753), Rotation2d.fromDegrees(-55)));

            private final Pose2d pose;
            public static io.vavr.collection.List<Pose2d> reefLeft = io.vavr.collection.List.of(A.getPose2d(), C.getPose2d(), F.getPose2d(), H.getPose2d(), J.getPose2d(), K.getPose2d());
            public static io.vavr.collection.List<Pose2d> reefRight = io.vavr.collection.List.of(B.getPose2d(), D.getPose2d(), E.getPose2d(), G.getPose2d(), I.getPose2d(), L.getPose2d());
            public static io.vavr.collection.List<Pose2d> reefPoses = io.vavr.collection.List.of(A.getPose2d(), B.getPose2d(), C.getPose2d(), D.getPose2d(), E.getPose2d(), F.getPose2d(), G.getPose2d(), H.getPose2d(), I.getPose2d(), J.getPose2d(), K.getPose2d(), L.getPose2d());
            public static io.vavr.collection.List<Pose2d> sourcePoses = io.vavr.collection.List.of(SOURCE_1.getPose2d(), SOURCE_2.getPose2d());

            private FieldStates(int id, boolean isRight) {
                Pose2d apriltag = APRIL_TAGS.get(id - 1).pose.toPose2d();
                Translation2d offset = new Translation2d(Units.inchesToMeters(29.0/2.0), Units.inchesToMeters(-6.25)).rotateBy(apriltag.getRotation());
                Translation2d fudgeFactor  = FUDGE_FACTOR.rotateBy(apriltag.getRotation());
                Translation2d leftRight = new Translation2d(0, Units.inchesToMeters(isRight ? 13.0 / 2 : -13.0 / 2)).rotateBy(apriltag.getRotation());
                this.pose = new Pose2d(apriltag.getX() + offset.getX() + fudgeFactor.getX() + leftRight.getX(), apriltag.getY() + offset.getY() + fudgeFactor.getY() + leftRight.getY(), apriltag.getRotation().plus(Rotation2d.k180deg));
            }

            private FieldStates(Pose2d pose) {
                this.pose = pose;
            }

            public Pose2d getPose2d() {
                return this.pose;
            }

            public Translation2d getTranslation2d() {
                return pose.getTranslation();
            }

            public Rotation2d getRotation2d() {
                return pose.getRotation();
            }
        }


        public static Pose2d allianceFlip(Pose2d pose) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flip(pose);
            }
            return pose;
        }

        public static Pose2d allianceFlipRotationally(Pose2d pose) {
            if (Robot.getAlliance() == Alliance.Red) {
                return flipRotationally(pose);
            }
            return pose;
        }

        public static List<Pose2d> allianceFlip(List<Pose2d> poses) {
            return poses.stream().map(pose -> allianceFlipRotationally(pose)).collect(Collectors.toList());
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

        public static Pose2d flipRotationally(Pose2d pose) {
            return new Pose2d(flipTranslationRotationally(pose.getTranslation()), Rotation2d.fromDegrees(pose.getRotation().getDegrees() + 180));
        }

        public static Translation2d flipTranslation(Translation2d translation) {
            return new Translation2d (
                FIELD_X_LENGTH - translation.getX(),
                translation.getY()
            );
        }

        public static Translation2d flipTranslationRotationally(Translation2d translation) {
            return new Translation2d (
                FIELD_X_LENGTH - translation.getX(),
                FIELD_Y_LENGTH - translation.getY()
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
            Rotation2d rotation = Rotation2d.fromDegrees(DriveConstants.controllerPOVOffset);
            if(Robot.getAlliance() == Alliance.Red || !fieldRelative) {
                rotation = Rotation2d.fromDegrees(DriveConstants.controllerPOVOffset * -1);
            }
            return translation.rotateBy(rotation);
        }
    }

    public static class LedConstants{

        public static final int CANDLE_ID = 14;

        public static final int WHITE_VALUE = 0; //leds used don't have a white value
        public static final double r_SPEED = 0.75;
        public static final double c_SPEED = 1;

        public static final int STARTING_ID = 8;
        public static final int NUM_LED = 27;
        public static final int TOTAL_LEDS = STARTING_ID + NUM_LED;

        public static final int SPARKING = 1;
        public static final double COOLING = 0.3;
        
        public static final double HOLDING_SPEED = 2;

        public static final double BRIGHTNESS = 1;

    }

    public static class IntakeConstants {
    
        public static final int PIVOT_LEADER_ID = 40;

        //90/12.38
        public static final double PIVOT_GEAR_RATIO = 90/11.785769;
        public static final double PIVOT_SAMPLE_PER_MINUTE = 60;
        public static final int PIVOT_STATOR_CURRENT_LIMIT = 40;
        public static final boolean PIVOT_INVERT = true;
        public static final Neutral PIVOT_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames PIVOT_STATUS_FRAME = StatusFrames.POSITION;

        public static final double PIVOT_POSITION_MIN = 0;
        public static final double PIVOT_POSITION_MAX = 180;
        public static final double PIVOT_TOLERANCE = 1;

        public static final int ROLLER_LEADER_ID = 41;

        public static final double ROLLER_GEAR_RATIO = 1;
        public static final double ROLLER_SAMPLE_PER_MINUTE = 60;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 40;
        public static final boolean ROLLER_INVERT = true;
        public static final Neutral ROLLER_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames ROLLER_STATUS_FRAME = StatusFrames.POSITION;

        public static final double ROLLER_pose_MIN = 0;
        public static final double ROLLER_pose_MAX = 1;
        public static final double ROLLER_TOLERANCE = 0.01;

        public static final int FIRST_SENSOR_ID = 0;

    }

    public static class ElevatorConstants {

        public static final int ELEVATOR_LEFT_ID = 30;
        public static final int ELEVATOR_RIGHT_ID = 31;

        //1/14 gear ratio based on wom
        // Units.inchesToMeters(62.625) / 67.5
        public static final double ELEVATOR_GEAR_RATIO = Units.inchesToMeters(60.4375)/40.18735;
        public static final double ELEVATOR_SAMPLE_PER_MINUTE = 60;
        public static final int ELEVATOR_STATOR_CURRENT_LIMIT = 60;
        public static final boolean ELEVATOR_INVERT = false;
        public static final Neutral ELEVATOR_NEUTRAL_MODE = Neutral.COAST;
        public static final StatusFrames ELEVATOR_STATUS_FRAME = StatusFrames.POSITION;

        public static final double ELEVATOR_POSITION_MIN = 0;
        public static final double ELEVATOR_POSITION_MAX = Units.inchesToMeters(60.4375);
        public static final double ELEVATOR_TOLERANCE = 0.01;
    }

    public static class ClimberConstants {
        
        public static final int CLIMBER_WINCH_ID = 20;
        public static final int CLIMB_ROLLER_ID = 21;


        public static final double CLIMBER_GEAR_RATIO = (90.0 / 450.0) * (5.0 / 3.0);
        public static final double CLIMBER_SAMPLE_PER_MINUTE = 60;
        public static final int CLIMBER_STATOR_CURRENT_LIMIT = 40;
        public static final boolean CLIMBER_INVERT = false;
        public static final Neutral CLIMBER_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames CLIMBER_STATUS_FRAME = StatusFrames.POSITION;

        public static final double CLIMBER_POSITiON_MIN = 9;
        public static final double CLIMBER_POSITION_MAX = 110;
        public static final double CLIMBER_TOLERANCE = 1;

        public static final double CLIMB_ROLLER_GEAR_RATIO = 1;
        public static final double CLIMB_ROLLER_SAMPLE_PER_MINUTE = 60;
        public static final int CLIMB_ROLLER_STATOR_CURRENT_LIMIT = 40;
        public static final boolean CLIMB_ROLLER_INVERT = false;
        public static final Neutral CLIMB_ROLLER_NEUTRAL_MODE = Neutral.BRAKE;

    }

    public static class ManipulatorConstants {
        public static final int ROLLER_LEADER_ID = 50;

        public static final double ROLLER_GEAR_RATIO = 1;
        public static final double ROLLER_SAMPLE_PER_MINUTE = 60;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 60;
        public static final boolean ROLLER_INVERT = true;
        public static final Neutral ROLLER_NEUTRAL_MODE = Neutral.BRAKE;
        public static final StatusFrames ROLLER_STATUS_FRAME = StatusFrames.DEFAULT;

        public static final int FIRST_SENSOR_ID = 0;
        public static final int SECOND_SENSOR_ID = 1;
    }
}