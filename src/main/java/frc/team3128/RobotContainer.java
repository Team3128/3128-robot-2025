package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import common.hardware.camera.Camera;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

    public static NAR_XboxController controller, controller2;

    private NarwhalDashboard dashboard;

    private final Command swerveDriveCommand;

    private Swerve swerve;

    public RobotContainer() {
        NAR_CANSpark.maximumRetries = 2;
        NAR_TalonFX.maximumRetries = 2;

        NAR_Shuffleboard.WINDOW_WIDTH = 10;

        controller = new NAR_XboxController(2);
        controller2 = new NAR_XboxController(3);
        
        swerve = Swerve.getInstance();
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX, controller::getLeftY, controller::getRightX);

        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);

        initCameras();
        initDashboard();
        configureButtonBindings();
    }   
    public double inchesToMeters(double inches){
        return Distance.ofRelativeUnits(inches, Units.Inches).in(Units.Meters);
    }

    public double degreesToRadians(double degrees){
        return Angle.ofRelativeUnits(degrees, Units.Degrees).in(Units.Radians);
    }

    private void configureButtonBindings() {
       
    }

    public void initCameras() {
         //update this later 
         Camera2.setResourcesCustom(()->swerve.getYaw(), (pose, time) -> swerve.addVisionMeasurement(pose, time), "src/main/java/frc/team3128/AprilTags2025.json", () -> swerve.getPose());
         Camera2.setThresholds(5,  10);
         if(Robot.isReal()){
            Camera2 frontRightCamera = new Camera2("FRONT_RIGHT", inchesToMeters(10.055), inchesToMeters(9.79), degreesToRadians(30), degreesToRadians(-28.125), 0);
            Camera2 frontLeftCamera = new Camera2("FRONT_LEFT", inchesToMeters(10.055), -inchesToMeters(9.79), degreesToRadians(-30), degreesToRadians(-28.125), 0);
            Camera2 backRightCamera = new Camera2("BACK_RIGHT", 0, 0, 0, 0, 0);
            Camera2 backLeftCamera = new Camera2("BACK_LEFT", 0, 0, 0, 0, 0);
         }  
    }

    public void initDashboard() {

    }
}
