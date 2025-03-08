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

import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Swerve;
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
    private Swerve swerve;
    private Intake intake;

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    private final Command swerveDriveCommand;

    public static Limelight limelight;


    public RobotContainer() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        buttonPad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);
        
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        DriverStation.silenceJoystickConnectionWarning(true);
        // initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {
        // buttonPad.getButton(1).onTrue(runOnce(()-> swerve.setBrakeMode(false)).ignoringDisable(true)).onFalse(runOnce(()-> swerve.setBrakeMode(true)).ignoringDisable(true));
        // buttonPad.getButton(2).onTrue(swerve.identifyOffsetsCommand());
        controller.getButton(XboxButton.kY).onTrue(runOnce(()-> swerve.resetGyro(0)));
        // controller.getButton(kLeftStick).onTrue(runOnce(()-> swerve.zeroLock()).andThen(runOnce(()-> swerve.resetEncoders())));

        controller.getButton(XboxButton.kA).onTrue(sequence(
            runOnce(()->Intake.rollerMotor.set(0.6)),
            runOnce(()->Intake.serial.set(0.4))
        )).onFalse(sequence(
            runOnce(()->Intake.rollerMotor.set(0)),
            runOnce(()->Intake.serial.set(0))
        ));

        controller.getButton(XboxButton.kB).onTrue(sequence(
            runOnce(()->Intake.rollerMotor.set(-0.6)),
            runOnce(()->Intake.serial.set(-0.4))
        )).onFalse(sequence(
            runOnce(()->Intake.rollerMotor.set(0)),
            runOnce(()->Intake.serial.set(0))
        ));
            
        // controller.getUpPOVButton().onTrue(runOnce(()-> swerve.snapToSource()));
        // controller.getDownPOVButton().onTrue(runOnce(()-> swerve.setPose(FieldStates.PROCESSOR.getPose2d())));
        // controller.getRightPOVButton().onTrue(runOnce(()-> swerve.snapToReef(true)));
        // controller.getLeftPOVButton().onTrue(runOnce(()-> swerve.snapToReef(false)));

        // controller.getButton(kRightStick).onTrue(runOnce(()-> swerve.snapToAngle()));
        // controller.getButton(kLeftStick).onTrue(runOnce(()-> swerve.resetGyro(0)));

    }

    public void initCameras() {
        Log.info("tags", APRIL_TAGS.get(0).toString());
        Camera.setResources(() -> swerve.getYaw(), (pose, time) -> swerve.addVisionMeasurement(pose, time), new AprilTagFieldLayout(APRIL_TAGS, FIELD_X_LENGTH, FIELD_Y_LENGTH), () -> swerve.getPose());
        // Camera.setThresholds(5,  10);
        if (Robot.isReal()) {
            // Camera frontRightCamera = new Camera("FRONT_RIGHT", Units.inchesToMeters(10.055), Units.inchesToMeters(9.79), Units.degreesToRadians(30), Units.degreesToRadians(-28.125), 0);
            Camera frontLeftCamera = new Camera("FRONT_LEFT", Units.inchesToMeters(13.5), -Units.inchesToMeters(0), Units.degreesToRadians(0), Units.degreesToRadians(0), 0);
            // Camera backRightCamera = new Camera("BACK_RIGHT", -Units.inchesToMeters(10.055), Units.inchesToMeters(9.79),  Units.degreesToRadians(150), Units.degreesToRadians(-28.125), 0);
            // Camera backLeftCamera = new Camera("BACK_LEFT", -Units.inchesToMeters(10.055), -Units.inchesToMeters(9.79), Units.degreesToRadians(-150), Units.degreesToRadians(-28.125), 0);
        }
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
    }
}
