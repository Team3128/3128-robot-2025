package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.limelight.Limelight;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import common.hardware.input.NAR_ButtonBoard;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Elevator.ElevatorStates;
// import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Manipulator.Manipulator;
import frc.team3128.subsystems.Robot.RobotManager;
import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.io.IOException;

import javax.lang.model.element.NestingKind;

import static edu.wpi.first.wpilibj2.command.Commands.*;


/**
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {

    public static final boolean printStatus = true;

    // Create all subsystems
    private RobotManager robot;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    // private Swerve swerve;
    // private final Command swerveDriveCommand;

    public static Limelight limelight;

    public RobotContainer() {
        // swerve = Swerve.getInstance();
        
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        buttonPad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);
        
        // swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        // CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        robot = RobotManager.getInstance();

        //uncomment line below to enable driving
        // CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {
        controller.initShuffleboard();
        buttonPad.getButton(1).whileTrue(robot.setStateCommand(IDLE)).onFalse(robot.setStateCommand(NEUTRAL));

        controller.getButton(kA).onTrue(robot.getCoralState(RPL1, RSL1));
        controller.getButton(kB).onTrue(robot.getCoralState(RPL2, RSL2));
        controller.getButton(kX).onTrue(robot.getCoralState(RPL3, RSL3));
        controller.getButton(kY).onTrue(robot.getCoralState(RPL4, RSL4));




        controller.getButton(kLeftTrigger).onTrue(robot.getAlgaeState(INTAKE));
        controller.getButton(kLeftBumper).onTrue(robot.getAlgaeState(EJECT_OUTTAKE));
        controller.getButton(kBack).onTrue(robot.getAlgaeState(PROCESSOR_PRIME, PROCESSOR_OUTTAKE));

        controller.getButton(kRightTrigger).onTrue(robot.setStateCommand(NEUTRAL));
        controller.getButton(kRightBumper).onTrue(robot.getClimbState());
        controller.getButton(kStart).onTrue(
            either(
                robot.setStateCommand(CLIMB_WINCH), 
                either(
                    robot.setStateCommand(INDEXING), 
                    robot.setStateCommand(SOURCE).onlyIf(() -> !Manipulator.getInstance().hasObjectPresent()), 
                    ()-> robot.stateEquals(SOURCE)), 
                ()-> robot.stateEquals(CLIMB_LOCK))
        );

        // controller.getUpPOVButton().onTrue(runOnce(()-> swerve.snapToSource()));
        // controller.getDownPOVButton().onTrue(runOnce(()-> swerve.setPose(FieldStates.PROCESSOR.getPose2d())));
        // controller.getRightPOVButton().onTrue(runOnce(()-> swerve.snapToReef(true)));
        // controller.getLeftPOVButton().onTrue(runOnce(()-> swerve.snapToReef(false)));

        // controller.getButton(kRightStick).onTrue(runOnce(()-> swerve.snapToAngle()));
        // controller.getButton(kLeftStick).onTrue(runOnce(()-> swerve.resetGyro(0)));

        new Trigger(()-> robot.stateEquals(INDEXING)).and(()-> Manipulator.getInstance().hasObjectPresent()).onTrue(robot.setStateCommand(NEUTRAL));
    }

    public void initCameras() {
        
    }

    public void initDashboard() {

    }
}
