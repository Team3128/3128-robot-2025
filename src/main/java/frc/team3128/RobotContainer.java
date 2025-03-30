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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.Constants.DriveConstants;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.autonomous.AutoPrograms;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Climber.RollerMechanism;
import frc.team3128.subsystems.Climber.WinchMechanism;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Manipulator.Manipulator;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;

import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.io.IOException;
import java.util.function.BooleanSupplier;

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

    private BooleanSupplier gyroReset;
    private BooleanSupplier elevReset;

    // private WinchMechanism winch;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    private final Command swerveDriveCommand;

    public static Limelight limelight;

    public static BooleanSupplier shouldRam;
    public static BooleanSupplier shouldPreClimb;


    @SuppressWarnings("resource")
    public RobotContainer() {
        swerve = Swerve.getInstance();
        // winch = WinchMechanism.getInstance();
        
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        buttonPad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);

        gyroReset = ()-> !new DigitalInput(9).get();
        elevReset = ()-> !new DigitalInput(8).get();
        
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        robot = RobotManager.getInstance();
        elevator = ElevatorMechanism.getInstance();


        AutoPrograms.getInstance();
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {
        buttonPad.getButton(12).onTrue(swerve.identifyOffsetsCommand().ignoringDisable(true));

        shouldRam = ()-> !buttonPad.getButton(1).getAsBoolean();
        shouldPreClimb = ()-> !buttonPad.getButton(2).getAsBoolean();


        controller2.getButton(kA).onTrue(Climber.getInstance().runCommand(0.8)).onFalse(Climber.getInstance().stopCommand());
        controller2.getButton(kB).onTrue(Climber.getInstance().runCommand(-0.8)).onFalse(Climber.getInstance().stopCommand());
        controller2.getButton(kX).onTrue(Climber.getInstance().resetCommand());


        controller.getButton(kA).onTrue(robot.getTempToggleCommand(RPL1, RSL1));
        controller.getButton(kB).onTrue(robot.getTempToggleCommand(RPL2, RSL2));
        controller.getButton(kX).onTrue(robot.getTempToggleCommand(RPL3, RSL3));
        controller.getButton(kY).onTrue(robot.getTempToggleCommand(RPL4, RSL4));

        controller.getButton(kLeftTrigger).onTrue(robot.getToggleCommand(INTAKE));
        controller.getButton(kLeftBumper).onTrue(robot.getToggleCommand(EJECT_OUTTAKE));

        controller.getButton(kRightTrigger).onTrue(robot.setStateCommand(NEUTRAL));
        
        controller.getButton(kRightBumper).onTrue(robot.getToggleCommand(
            robot.setStateCommand(CLIMB_PRIME), 
            waitUntil(()-> RollerMechanism.getInstance().isCaptured()).andThen(robot.setStateCommand(CLIMB)),
            ()-> robot.stateEquals(CLIMB_PRIME))
        );

        controller.getButton(kRightStick).onTrue(runOnce(()-> swerve.resetGyro(0)));
        controller.getButton(kLeftStick).onTrue(swerve.autoAlignSource());

        controller.getButton(kBack).onTrue(sequence(
            swerve.autoAlign(false, shouldRam).andThen(() -> robot.autoScore())
            .beforeStarting(robot.setStateCommand(TELE_HOLD))
        ));

        controller.getButton(kStart).onTrue(sequence(
            swerve.autoAlign(true, shouldRam).andThen(() -> robot.autoScore())
            .beforeStarting(robot.setStateCommand(TELE_HOLD))
        ));

        controller.getDownPOVButton().onTrue(runOnce(()-> swerve.snapToElement()));

        // new Trigger(()-> robot.stateEquals(RobotStates.CLIMB_PRIME)).and(()-> Climber.getInstance().lc.getMeasurement().distance_mm < 100).debounce(2).onTrue(robot.setStateCommand(CLIMB));

        new Trigger(gyroReset).and((()-> DriverStation.isDisabled())).onTrue(runOnce(() -> swerve.resetGyro(0)).ignoringDisable(true));
        new Trigger(elevReset).and((() -> DriverStation.isDisabled())).onTrue(elevator.resetCommand().ignoringDisable(true));
    }

    public void initCameras() {
        Log.info("tags", APRIL_TAGS.get(0).toString());
        Camera.setResources(() -> swerve.getYaw(), (pose, time) -> swerve.addVisionMeasurement(pose, time), new AprilTagFieldLayout(APRIL_TAGS, FIELD_X_LENGTH, FIELD_Y_LENGTH), () -> swerve.getPose());
        Camera.addIgnoredTags(4, 5, 14, 15);
        if (Robot.isReal()) {
            Camera backRightCamera = new Camera("BOTTOM_RIGHT", 0.27, -0.27,  Units.degreesToRadians(-15), 0, 0);
            backRightCamera.setThresholds(0.3, 3, 0.3);
            
            Camera backLeftCamera = new Camera("BOTTOM_LEFT", 0.09, 0.145, 0, 0, 0);
            backLeftCamera.setThresholds(0, 3, 0.3);
        }
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
    }
}