package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.limelight.Limelight;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import common.hardware.input.NAR_ButtonBoard;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.subsystems.Swerve;


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

    // private NAR_ButtonBoard judgePad;
    private NAR_ButtonBoard buttonPad;

    public static NAR_XboxController controller;
    public static NAR_XboxController controller2;

    private NarwhalDashboard dashboard;
    private Swerve swerve;
    private final Command swerveDriveCommand;

    public static Limelight limelight;

    private final Command swerveDriveCommand;

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
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX,controller::getLeftY, controller::getRightX);

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   

    private void configureButtonBindings() {

    }

    public void initCameras() {

    }

    public void initDashboard() {

    }
}
