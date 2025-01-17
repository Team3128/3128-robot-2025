package frc.team3128.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
// import frc.team3128.subsystems.Swerve;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;

import static frc.team3128.Constants.SwerveConstants.*;


/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Lucas Han
 */

public class AutoPrograms {

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();
    private HashMap<String, Command> pathMap = new HashMap<String, Command>();
    // private static Swerve swerve = Swerve.getInstance();
    private RobotConfig robotConfig;
    private static AutoPrograms instance;
    SendableChooser<Command> autoChooser;

    public AutoPrograms() {
        configPathPlanner();
        initAutoSelector();
        autoChooser = new SendableChooser<Command>();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static synchronized AutoPrograms getInstance() {
        if (instance == null) instance = new AutoPrograms();
        return instance;
    }

    private void initAutoSelector() {
        List<String> autoStrings = AutoBuilder.getAllAutoNames();
        
        for (String autoName : autoStrings) {
            try {
                for (PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(autoName)) {
                    pathMap.put(path.name, AutoBuilder.followPath(path));
                }
            } catch (Exception e) {}
        }
    }

    private void configPathPlanner() {
        Pathfinding.setPathfinder(new LocalADStar());

        // try {
        //     robotConfig = RobotConfig.fromGUISettings();
        // } catch (Exception e) {
        //     robotConfig = new RobotConfig(
        //         ROBOT_MASS,
        //         ROBOT_MOI, 
        //         new ModuleConfig(
        //             DRIVE_WHEEL_DIAMETER / 2, 
        //             MAX_DRIVE_SPEED, 
        //             WHEEL_COF, 
        //             DCMotor.getKrakenX60(1),
        //             DRIVE_MOTOR_GEAR_RATIO, 
        //             (double) DRIVE_MOTOR_CURRENT_LIMIT, 
        //             1
        //         ),
        //         Swerve.moduleOffsets
        //     );
        // }

        // AutoBuilder.configure(
        //     swerve::getPose, 
        //     swerve::resetOdometry, 
        //     swerve::getRobotVelocity, 
        //     (velocity, feedforwards)-> swerve.drive(velocity), 
        //     new PPHolonomicDriveController(
        //         new PIDConstants(Swerve.translationConfig.kP, Swerve.translationConfig.kI, Swerve.translationConfig.kD),
        //         new PIDConstants(Swerve.rotationConfig.kP, Swerve.rotationConfig.kI, Swerve.rotationConfig.kD)
        //     ),
        //     robotConfig,
        //     ()-> Robot.getAlliance() == Alliance.Red,
        //     swerve
        //     );
    }

    public static Command getPathPlannerAuto(String trajectoryName) {
        return AutoBuilder.buildAuto(trajectoryName);
    }  

    public static Command getPathPlannerPath(String name) throws Exception {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(name));
    }

    public Command getAuto(String name) {
        return autoMap.get(name);
    }

    public Command getPath(String name) {
        return pathMap.get(name);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = ""; //NarwhalDashboard.getInstance().getSelectedAuto();
        String hardcode = "";
        
        Command autoCommand;
        if (selectedAutoName == null) {
            selectedAutoName = hardcode;
        }
        else if (selectedAutoName.equals("default")) {
            defaultAuto();
        }
        autoCommand = autoMap.get(selectedAutoName);

        Log.info("AUTO_SELECTED", selectedAutoName);
        return autoCommand.beforeStarting(reset());
    }

    private Command defaultAuto(){
        return none();
    }

    private Command reset() {
        return none();
    }
}