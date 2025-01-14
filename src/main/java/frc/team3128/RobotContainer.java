package frc.team3128;


import common.core.misc.NAR_Robot;
import common.hardware.input.NAR_XboxController;
import common.hardware.limelight.Limelight;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import common.hardware.input.NAR_ButtonBoard;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
<<<<<<< Updated upstream
=======
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.doglog.DogLog;
import frc.team3128.subsystems.Intake;
>>>>>>> Stashed changes
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

    public static Limelight limelight;

    private final Command swerveDriveCommand;

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 3;
        NAR_TalonFX.maximumRetries = 1;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        // judgePad = new NAR_ButtonBoard(1);
        controller = new NAR_XboxController(2);
        buttonPad = new NAR_ButtonBoard(3);
        controller2 = new NAR_XboxController(4);

        swerve = Swerve.getInstance();
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX,controller::getLeftY, controller::getRightX);

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
<<<<<<< Updated upstream
        initDashboard();
    }   

    private void configureButtonBindings() {

=======
        NAR_Robot.addPeriodic(()->{logData();}, 0.02);

    } 
    private void logData() {
        DogLog.log("Test", "Hello");
        swerve.initDogLog();
    }  

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kA)
            .onTrue(intake.run(-0.30))
            .onFalse(intake.stop());
        controller.getButton(XboxButton.kB)
            .onTrue(intake.run(-0.15))
            .onFalse(intake.stop());
        controller.getButton(XboxButton.kX)
            .onTrue(Commands.runOnce(()->swerve.resetGyro(0)));
>>>>>>> Stashed changes
    }

    public void initCameras() {

    }

    public void initDashboard() {

    }
}
