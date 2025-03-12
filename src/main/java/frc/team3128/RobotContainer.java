package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.limelight.Limelight;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import common.hardware.camera.Camera;
import common.hardware.input.NAR_ButtonBoard;
import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import common.utility.sysid.CmdSysId;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.commands.CmdAlignReef;
import frc.team3128.autonomous.AutoPrograms;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.team3128.autonomous.AutoPrograms;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Climber.WinchMechanism;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Manipulator.Manipulator;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;
import frc.team3128.subsystems.Led.Led;
import frc.team3128.subsystems.Led.LedStates;

import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.io.IOException;

import javax.lang.model.element.NestingKind;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.VisionConstants.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;

/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

    // Create all subsystems
    private RobotManager robot;
    private ElevatorMechanism elevator;
    // private Manipulator manipulator;
    private Swerve swerve;
    private Led led;

    // private WinchMechanism winch;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    private final Command swerveDriveCommand;

    public static Limelight limelight;


    public RobotContainer() {
        swerve = Swerve.getInstance();
        // winch = WinchMechanism.getInstance();
        
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        buttonPad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);
        
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        robot = RobotManager.getInstance();
        elevator = ElevatorMechanism.getInstance();
        // manipulator = Manipulator.getInstance();

        //uncomment line below to enable driving
        // CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        NAR_Shuffleboard.addSendable("RobotContainer", "NEUTRAL", robot, 0, 0).withWidget(BuiltInWidgets.kToggleSwitch);

        AutoPrograms.getInstance();
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {
        // buttonPad.getButton(1).whileTrue(runOnce(()-> swerve.setBrakeMode(false))).onFalse(runOnce(()-> swerve.setBrakeMode(true)));
        buttonPad.getButton(2).onTrue(swerve.identifyOffsetsCommand().ignoringDisable(true));
        // buttonPad.getButton(3).onTrue(runOnce(()-> robot.setNeutralMode(Neutral.COAST))).onFalse(runOnce(()-> robot.setNeutralMode(Neutral.BRAKE)));
        // buttonPad.getButton(4).onTrue(Climber.getInstance().resetCommand().ignoringDisable(true));
        // buttonPad.getButton(1).whileTrue(runOnce(()->Swerve.fieldRelative = false)).whileFalse(runOnce(()->Swerve.fieldRelative = true));

        controller2.getButton(kA).onTrue(Climber.getInstance().runCommand(0.8)).onFalse(Climber.getInstance().stopCommand());
        controller2.getButton(kB).onTrue(Climber.getInstance().runCommand(-0.8)).onFalse(Climber.getInstance().stopCommand());
        controller2.getButton(kX).onTrue(Climber.getInstance().resetCommand());


        controller.getButton(kA).onTrue(robot.getTempToggleCommand(RPL1, RSL1));
        controller.getButton(kB).onTrue(robot.getTempToggleCommand(RPL2, RSL2));
        controller.getButton(kX).onTrue(robot.getTempToggleCommand(RPL3, RSL3));
        controller.getButton(kY).onTrue(robot.getTempToggleCommand(RPL4, RSL4));

        controller.getButton(kLeftTrigger).onTrue(robot.getToggleCommand(INTAKE));
        controller.getButton(kLeftBumper).onTrue(robot.getToggleCommand(EJECT_OUTTAKE));
        controller.getButton(kBack).onTrue(robot.getToggleCommand(HIGH_INTAKE));

        controller.getButton(kRightTrigger).onTrue(robot.setStateCommand(NEUTRAL));
        controller.getButton(kRightBumper).onTrue(robot.getToggleCommand(CLIMB_PRIME, CLIMB));
        controller.getButton(kStart).onTrue(robot.setStateCommand(FULL_NEUTRAL));

        controller.getButton(kRightStick).onTrue(runOnce(()-> swerve.resetGyro(0)));
        controller.getButton(kLeftStick).onTrue(runOnce(()-> swerve.snapToElement()));

        // controller2.getButton(kX).onTrue(
        //     swerve.characterizeTranslation(0, 1, 10))
        // );
        // controller2.getButton(kY).onTrue(
        //     swerve.characterizeRotation(0, 1, 10))
        // );

       // controller.getRightPOVButton().onTrue(runOnce(()-> swerve.zeroLock()));
        // controller.getLeftPOVButton().onTrue(swerve.autoAlign(false));
        controller.getUpPOVButton().onTrue(AutoPrograms.getInstance().pathToPose(swerve.getPose().nearest(allianceFlip(FieldStates.sourcePoses.asJava()))));
        controller.getRightPOVButton().onTrue(sequence(
            runOnce(() -> swerve.setThrottle(0.3)),
            runOnce(()-> swerve.pathToReef(true)),
            runOnce(()-> Swerve.autoEnabled = true),
            waitUntil(()-> swerve.atTranslationSetpoint()).withTimeout(1.5),
            runOnce(()-> swerve.snapToElement()),
            waitUntil(()-> swerve.atRotationSetpoint()).withTimeout(1.5),
            runOnce(()->swerve.angleLock(90)),
            // waitSeconds(1),
            runOnce(()-> {
                Camera.disableAll();
                Swerve.autoEnabled = false;
                swerve.moveBy(new Translation2d(FUDGE_FACTOR.getX(), 0).rotateBy(swerve.getClosestReef().getRotation()));
            }),
            waitUntil(() -> swerve.atTranslationSetpoint()).withTimeout(1.5).finallyDo(()-> Swerve.disable())
        ).finallyDo(() -> {
            Camera.enableAll();
            swerve.setThrottle(1);
        }));
        controller.getLeftPOVButton().onTrue(sequence(
            runOnce(() -> swerve.setThrottle(0.3)),
            runOnce(()-> swerve.pathToReef(false)),
            runOnce(()-> Swerve.autoEnabled = true),
            waitUntil(()-> swerve.atTranslationSetpoint()).withTimeout(1.5),
            runOnce(()->swerve.angleLock(90)),
            // waitSeconds(1),
            runOnce(()-> {
                Camera.disableAll();
                Swerve.autoEnabled = false;
                swerve.moveBy(new Translation2d(FUDGE_FACTOR.getX(), 0).rotateBy(swerve.getClosestReef().getRotation()));
            }),
            waitUntil(() -> swerve.atTranslationSetpoint()).withTimeout(1.5).finallyDo(()-> Swerve.disable())
        ).finallyDo(() -> {
            Camera.enableAll();
            swerve.setThrottle(1);
        }));
    }

    public void initCameras() {
        Log.info("tags", APRIL_TAGS.get(0).toString());
        Camera.setResources(() -> swerve.getYaw(), (pose, time) -> swerve.addVisionMeasurement(pose, time), new AprilTagFieldLayout(APRIL_TAGS, FIELD_X_LENGTH, FIELD_Y_LENGTH), () -> swerve.getPose());
        Camera.setThresholds(0.3, 3, 0.3);
        if (Robot.isReal()) {
            Camera backRightCamera = new Camera("BOTTOM_RIGHT", 0.27, -0.27,  Units.degreesToRadians(-15), 0, 0);
            // backRightCamera.setThresholds(0.3, 1.5, 0.3);
            Camera backLeftCamera = new Camera("BOTTOM_LEFT", 0.09, 0.145, 0, 0, 0);
            // backLeftCamera.setThresholds(0.3, 1.5, 0.3);
            // Camera topCamera = new Camera("TOP", -Units.inchesToMeters(6), -Units.inchesToMeters(12.5), Units.degreesToRadians(180), Units.degreesToRadians(-45), 0);
        }
    }

    public void initDashboard() {
        // dashboard = NarwhalDashboard.getInstance();
        // dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        // dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        // dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
    }
}