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
import frc.team3128.subsystems.Swerve;
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

    // Create all subsystems
    private RobotManager robot;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    private Swerve swerve;
    private final Command swerveDriveCommand;

    public static Limelight limelight;

    public RobotContainer() {
        swerve = Swerve.getInstance();
        
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        // judgePad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);
        
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        swerve = Swerve.getInstance();
        robot = RobotManager.getInstance();

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {

        buttonPad.getButton(1).whileTrue(robot.setStateCommand(IDLE)).onFalse(robot.setStateCommand(NEUTRAL));

        controller.getButton(kA).onTrue(robot.getCoralState(RPL1, RSL1));
        controller.getButton(kA).onTrue(robot.getCoralState(RPL2, RSL2));
        controller.getButton(kA).onTrue(robot.getCoralState(RPL3, RSL3));
        controller.getButton(kA).onTrue(robot.getCoralState(RPL4, RSL4));
        controller.getButton(kA).onTrue(robot.getCoralState(NEUTRAL, SOURCE, ()-> true));

        controller.getButton(kLeftTrigger).onTrue(robot.getAlgeaState(INTAKE));
        controller.getButton(kLeftBumper).onTrue(robot.getAlgeaState(EJECT_OUTTAKE));
        controller.getButton(kBack).onTrue(robot.getAlgeaState(PROCESSOR_PRIME, PROCESSOR_OUTTAKE));

        controller.getButton(kRightTrigger).onTrue(robot.setStateCommand(NEUTRAL));

        // controller.getButton(kRightTrigger).toggleOnTrue(robot.setStateCommand(SOURCE)).onFalse();
    }

    public void initCameras() {
        
    }

    public void initDashboard() {

    }
}
