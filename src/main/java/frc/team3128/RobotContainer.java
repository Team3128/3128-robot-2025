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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.Constants.RobotConstants;
import frc.team3128.Constants.FieldConstants.FieldStates;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
// import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Manipulator.Manipulator;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;
import frc.team3128.subsystems.Manipulator.RollerMechanism;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;

import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.io.IOException;

import javax.lang.model.element.NestingKind;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

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

    private WinchMechanism winch;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    private final Command swerveDriveCommand;

    public static Limelight limelight;


    public RobotContainer() {
        swerve = Swerve.getInstance();
        winch = WinchMechanism.getInstance();
        
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
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {
        // buttonPad.getButton(1).whileTrue(runOnce(()-> swerve.setBrakeMode(false))).onFalse(runOnce(()-> swerve.setBrakeMode(true)));
        // buttonPad.getButton(2).onTrue(swerve.identifyOffsetsCommand().ignoringDisable(true));
        // buttonPad.getButton(3).onTrue(runOnce(()-> robot.setNeutralMode(Neutral.COAST))).onFalse(runOnce(()-> robot.setNeutralMode(Neutral.BRAKE)));
        buttonPad.getButton(4).onTrue(Climber.getInstance().resetCommand().ignoringDisable(true));

        controller.getButton(kA).onTrue(robot.getTempToggleCommand(RPL1, RSL1));
        controller.getButton(kB).onTrue(robot.getTempToggleCommand(RPL2, RSL2));
        controller.getButton(kX).onTrue(robot.getTempToggleCommand(RPL3, RSL3));
        controller.getButton(kY).onTrue(robot.getTempToggleCommand(RPL4, RSL4));

        controller.getButton(kLeftTrigger).onTrue(robot.getToggleCommand(INTAKE));
        controller.getButton(kLeftBumper).onTrue(robot.getToggleCommand(EJECT_OUTTAKE));
        controller.getButton(kBack).onTrue(robot.getTempToggleCommand(PROCESSOR_PRIME, PROCESSOR_OUTTAKE));

        controller.getButton(kRightTrigger).onTrue(robot.setStateCommand(NEUTRAL));
        controller.getButton(kRightBumper).onTrue(robot.getToggleCommand(CLIMB_PRIME, CLIMB));
        controller.getButton(kStart).onTrue(robot.getToggleCommand(CLIMB_PRIME, CLIMB));

        controller.getButton(kRightStick).onTrue(runOnce(()-> swerve.resetGyro(0)));
        controller.getButton(kLeftStick).onTrue(runOnce(()-> swerve.resetEncoders()));

        // controller2.getButton(kX).onTrue(
        //     swerve.characterize(0, 1, 10)
        //         .beforeStarting(() -> swerve.zeroLock())
        // );
        // controller2.getButton(kY).onTrue(
        //     swerve.characterize(0, 1, 10)
        //         .beforeStarting(() -> swerve.oLock())
        // );

        controller2.getButton(kA).onTrue(winch.runCommand(0.8)).onFalse(winch.runCommand(0));
        controller2.getButton(kB).onTrue(winch.runCommand(-0.8)).onFalse(winch.runCommand(0));
        controller2.getButton(kX).onTrue(winch.resetCommand());
        controller2.getButton(kY).onTrue(Climber.getInstance().setStateCommand(ClimberStates.CLIMB_PRIME));
        controller2.getButton(kRightBumper).onTrue(Climber.getInstance().setStateCommand(ClimberStates.CLIMB));

        new Trigger(()-> Elevator.getInstance().stateEquals(ElevatorStates.NEUTRAL)).and(()-> elevator.atSetpoint()).debounce(5).onTrue(Elevator.getInstance().resetCommand());
        // new Trigger(()-> !RobotManager.getInstance().stateEquals(NEUTRAL)).onTrue(runOnce(()->  Swerve.getInstance().throttle = RobotConstants.slow)).onFalse(runOnce(()->  Swerve.getInstance().throttle = RobotConstants.fast));
        // controller.getUpPOVButton().onTrue(runOnce(()-> swerve.snapToSource()));
        controller.getDownPOVButton().onTrue(runOnce(()-> swerve.moveTo(allianceFlip(FieldStates.REEF_1.getPose2d()).getTranslation())));
        controller.getUpPOVButton().onTrue(runOnce(()-> swerve.snapToAngle()));
        controller.getRightPOVButton().onTrue(runOnce(()-> swerve.pathToReef(true)));
        controller.getLeftPOVButton().onTrue(runOnce(()-> swerve.pathToReef(false)));
        // controller.getRightPOVButton().onTrue(runOnce(()-> swerve.snapToReef(true)));
        // controller.getLeftPOVButton().onTrue(runOnce(()-> swerve.snapToReef(false)));
    }

    public void initCameras() {
        Log.info("tags", APRIL_TAGS.get(0).toString());
        Camera.setResources(() -> swerve.getYaw(), (pose, time) -> swerve.addVisionMeasurement(pose, time), new AprilTagFieldLayout(APRIL_TAGS, FIELD_X_LENGTH, FIELD_Y_LENGTH), () -> swerve.getPose());
        Camera.setThresholds(0.35, 3, 10);
        if (Robot.isReal()) {
            Camera backRightCamera = new Camera("BOTTOM_RIGHT", Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79),  0, Units.degreesToRadians(-28.125), 0);
            Camera backLeftCamera = new Camera("BOTTOM_LEFT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), 0, Units.degreesToRadians(-28.125), 0);
            // Camera topCamera = new Camera("TOP", -Units.inchesToMeters(6), -Units.inchesToMeters(12.5), Units.degreesToRadians(180), Units.degreesToRadians(-45), 0);
        }
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
    }
}
