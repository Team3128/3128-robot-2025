package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.subsystems.Intake;
import frc.team3128.subsystems.Winch;
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
    

    public static NAR_XboxController controller, controller2;

    private NarwhalDashboard dashboard;
    private Swerve swerve;
    private Intake intake;
    private Winch winch;

    private final Command swerveDriveCommand;

    public RobotContainer() {
        swerve = Swerve.getInstance();
        intake = Intake.getInstance();
        winch = Winch.getInstance();
        
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);
        
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        // initCameras();
        // initDashboard();
        configureButtonBindings();
    }   

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kA)
            .onTrue(intake.run(-0.75))
            .onFalse(intake.stop());
        controller.getButton(XboxButton.kB)
            .onTrue(intake.run(0.75))
            .onFalse(intake.stop());

        controller.getButton(XboxButton.kLeftTrigger)
            .onTrue(winch.run(-1))
            .onFalse(winch.stop());
        controller.getButton(XboxButton.kLeftBumper)
            .onTrue(winch.run(1))
            .onFalse(winch.stop());
        controller.getButton(XboxButton.kX)
            .onTrue(Commands.runOnce(()->swerve.resetGyro(0)));
    }

    public void initCameras() {

    }

    public void initDashboard() {

    }
}
