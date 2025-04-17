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
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Intake.PivotMechanism;
import frc.team3128.subsystems.Manipulator.Manipulator;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;
import edu.wpi.first.math.Pair;


import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import javax.lang.model.element.NestingKind;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.VisionConstants.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;



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

    private DigitalInput gyroReset;
    private DigitalInput elevReset;


    // private WinchMechanism winch;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    private final Command swerveDriveCommand;

    public static Limelight limelight;

    public static BooleanSupplier shouldRam = ()-> false;
    public static BooleanSupplier shouldPreClimb = ()-> false;
    // public static BooleanSupplier bargeAutoMovement = () -> true;
    public static BooleanSupplier shouldWait;

    public static BooleanSupplier allianceWrite = () -> false;
    public static BooleanSupplier allianceRead = () -> false;


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

        gyroReset = new DigitalInput(9);
        elevReset = new DigitalInput(8);
        
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        robot = RobotManager.getInstance();
        elevator = ElevatorMechanism.getInstance();

        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {
        buttonPad.getButton(9).onTrue(swerve.identifyOffsetsCommand().ignoringDisable(true));

        shouldRam = ()-> buttonPad.getButton(1).getAsBoolean();
        shouldPreClimb = ()-> !buttonPad.getButton(2).getAsBoolean();
        shouldWait = () -> !buttonPad.getButton(5).getAsBoolean();
        // bargeAutoMovement = () -> !buttonPad.getButton(3).getAsBoolean();
        // shouldRam = ()-> false;
        // shouldPreClimb = ()-> false;

        allianceWrite = ()-> buttonPad.getButton(12).getAsBoolean();
        allianceRead = ()-> buttonPad.getButton(11).getAsBoolean();

        buttonPad.getButton(11).onTrue(runOnce(()-> {
            if(buttonPad.getButton(12).getAsBoolean()) Robot.alliance = DriverStation.Alliance.Red;
            else Robot.alliance = DriverStation.Alliance.Blue;
            Log.info("Alliance", Robot.getAlliance().name());
        }).ignoringDisable(true));

        buttonPad.getButton(10).onTrue(runOnce(()-> Log.info("Alliance", Robot.getAlliance().toString())).ignoringDisable(true));



        controller2.getButton(kA).onTrue(WinchMechanism.getInstance().runCommand(0.5)).onFalse(WinchMechanism.getInstance().stopCommand());
        controller2.getButton(kB).onTrue(WinchMechanism.getInstance().runCommand(-0.5)).onFalse(WinchMechanism.getInstance().stopCommand());
        controller2.getButton(kX).onTrue(WinchMechanism.getInstance().resetCommand().ignoringDisable(true));
        controller2.getButton(kY).onTrue(robot.setStateCommand(FULL_NEUTRAL));


        controller.getButton(kA).onTrue(robot.getTempToggleCommand(RPL1, RSL1));
        controller.getButton(kB).onTrue(robot.getTempToggleCommand(RPL2, RSL2));
        controller.getButton(kX).onTrue(robot.getTempToggleCommand(RPL3, RSL3));
        controller.getButton(kY).onTrue(robot.getTempToggleCommand(RPL4, RSL4));

        controller.getButton(kLeftTrigger).onTrue(robot.getToggleCommand(INTAKE));
        controller.getButton(kLeftBumper).onTrue(robot.setStateCommand(OUTTAKE)).onFalse(robot.setStateCommand(NEUTRAL));

        // controller.getButton(kRightTrigger).onTrue(robot.setStateCommand(NEUTRAL));
        // controller.getButton(kRightBumper).onTrue(robot.getToggleCommand(CLIMB_PRIME, CLIMB));
        controller.getButton(kRightBumper).onTrue(either(
            robot.getToggleCommand(RSA1, RSA2),
            robot.alignAlgaeIntake(),
            ()-> buttonPad.getButton(4).getAsBoolean()
        ));
        controller.getButton(kRightTrigger).onTrue(either(
            sequence(
                robot.setStateCommand(RPB),
                waitSeconds(2),
                robot.setStateCommand(RSB),
                waitSeconds(0.5),
                robot.setStateCommand(NEUTRAL)
            ),
            robot.alignAlgaeScore(),
            ()-> !buttonPad.getButton(3).getAsBoolean()
        ));


        controller.getButton(kRightStick).onTrue(robot.setStateCommand(NEUTRAL));
        controller.getButton(kLeftStick).onTrue(robot.alignCoralIntake());

        controller.getButton(kBack).onTrue(robot.alignScoreCoral(false, shouldWait)
            .beforeStarting(robot.setStateCommand(TELE_HOLD)));

        controller.getButton(kStart).onTrue(robot.alignScoreCoral(true, shouldWait)
            .beforeStarting(robot.setStateCommand(TELE_HOLD)));


        controller.getUpPOVButton().onTrue(runOnce(()-> swerve.resetGyro(0)));
        controller.getRightPOVButton().onTrue(robot.getToggleCommand(CLIMB_PRIME, CLIMB));
        controller.getDownPOVButton().onTrue(runOnce(()-> swerve.snapToElement()));
        // controller.getUpPOVButton().onTrue(
        //     runOnce(()-> swerve.moveBy(new Translation2d(2, 0)))
        // );

        new Trigger(()-> !gyroReset.get()).and((()-> DriverStation.isDisabled())).onTrue(runOnce(() -> swerve.resetGyro(0)).ignoringDisable(true).andThen(Commands.print("Gyro Zeroed").ignoringDisable(true)).ignoringDisable(true));
        new Trigger(()-> !elevReset.get()).and((() -> DriverStation.isDisabled())).onTrue(elevator.resetCommand().ignoringDisable(true).andThen(PivotMechanism.getInstance().resetCommand().ignoringDisable(true).andThen(WinchMechanism.getInstance().resetCommand().ignoringDisable(true))).andThen(Commands.print("Subsystems Zeroed").ignoringDisable(true)).ignoringDisable(true));
        new Trigger(allianceWrite).onTrue(runOnce(()-> Robot.getAlliance()).ignoringDisable(true));
        new Trigger(()-> RollerMechanism.getInstance().isCaptured()).and(()-> robot.stateEquals(CLIMB_PRIME)).debounce(0.25).whileTrue(Commands.repeatingSequence(Commands.print("Captured"))).onFalse(Commands.print("Not Captured --------------------------------------------------------------------------------------------"));
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