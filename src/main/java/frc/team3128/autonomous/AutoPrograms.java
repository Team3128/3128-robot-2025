package frc.team3128.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.Constants.FieldConstants.FieldStates;
// import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import common.utility.Log;
import common.utility.narwhaldashboard.NarwhalDashboard;

import static frc.team3128.Constants.FieldConstants.allianceFlipRotationally;
import static frc.team3128.Constants.FieldConstants.flipRotationally;
// import static frc.team3128.Constants.FieldConstants.FieldStates.REEF_5;
// import static frc.team3128.Constants.FieldConstants.FieldStates.REEF_6;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;
import frc.team3128.subsystems.Robot.RobotStates;


/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Lucas Han
 */

public class AutoPrograms {

    private HashMap<String, Command> autoMap = new HashMap<String, Command>();
    private HashMap<String, Command> pathMap = new HashMap<String, Command>();
    private static Swerve swerve = Swerve.getInstance();
    private RobotConfig robotConfig;
    private static AutoPrograms instance;
    private RobotManager robot;
    SendableChooser<Command> autoChooser;

    private AutoPrograms() {
        robot = RobotManager.getInstance();

        configPathPlanner();
        initAutoSelector();
    }

    public static synchronized AutoPrograms getInstance() {
        if (instance == null) instance = new AutoPrograms();
        return instance;
    }


    public void initAutoSelector() {
        autoMap.clear();
        pathMap.clear();
        List<String> autoStrings = AutoBuilder.getAllAutoNames();

        for(String autoName: autoStrings) {
            autoMap.put(autoName, getPathPlannerAuto(autoName));
            try {
                for(PathPlannerPath path : PathPlannerAuto.getPathGroupFromAutoFile(autoName)) {
                    pathMap.put(path.name, AutoBuilder.followPath(path));
                }
            } catch(Exception e) {}
        }
        //autoMap.put("RB_3pc_EDC_auto", getPathPlannerAuto("RB_3pc_EDC_auto"));
        // NarwhalDashboard.getInstance().addAutos(autoStrings.toArray(new String[0]));
    }

    private void configPathPlanner() {
        Pathfinding.setPathfinder(new LocalADStar());

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            robotConfig = new RobotConfig(
                ROBOT_MASS,
                ROBOT_MOI, 
                new ModuleConfig(
                    DRIVE_WHEEL_DIAMETER / 2, 
                    4.1, 
                    WHEEL_COF, 
                    DCMotor.getKrakenX60(1),
                    DRIVE_MOTOR_GEAR_RATIO, 
                    DRIVE_MOTOR_CURRENT_LIMIT, 
                    1
                ),
                Swerve.moduleOffsets
            );
        }

        FlippingUtil.symmetryType = FieldSymmetry.kRotational;
        AutoBuilder.configure(
            swerve::getPose, 
            swerve::resetOdometry, 
            swerve::getRobotVelocity, 
            (velocity, feedforwards)-> swerve.drive(ChassisSpeeds.fromRobotRelativeSpeeds(velocity, swerve.getGyroRotation2d())), 
            new PPHolonomicDriveController(
                new PIDConstants(Swerve.translationConfig.kP, Swerve.translationConfig.kI, Swerve.translationConfig.kD),
                new PIDConstants(Swerve.rotationConfig.kP, Swerve.rotationConfig.kI, Swerve.rotationConfig.kD)
            ),
            robotConfig,
            ()-> Robot.getAlliance() == Alliance.Red,
            swerve
            );

        for (FieldStates state : FieldStates.values()) {
            // if (state.name().equals("D")){
            //     NamedCommands.registerCommand(
            //         "Score L4 " + state.name(),
            //         // none()
            //         parallel(
            //             run(() -> swerve.drive(0, 0, 0)),
            //             sequence(
            //                 waitSeconds(1.15),
            //                 robot.setStateCommand(AUTO_HOLD),
            //                 robot.setStateCommand(RPL4)
            //             )
            //         ).withDeadline(swerve.autoAlign(state.getPose2d(), false).
            //             andThen(() -> robot.autoScore()).
            //             andThen(waitUntil(() -> robot.stateEquals(NEUTRAL))))
            //     );
            // }
            if (state.name().length() == 1) {
                NamedCommands.registerCommand(
                    "Score L4 " + state.name(),
                    // none()
                    parallel(
                        run(() -> swerve.drive(0, 0, 0)),
                        sequence(
                            waitSeconds(0.8),
                            robot.setStateCommand(AUTO_HOLD),
                            robot.setStateCommand(RPL4)
                        )
                    ).withDeadline(robot.alignScoreCoral(FieldConstants.allianceFlipRotationally(state.getPose2d()), () -> false).
                        andThen(() -> robot.autoScore()).
                        andThen(waitUntil(() -> robot.stateEquals(NEUTRAL))))
                );
                NamedCommands.registerCommand(
                    "Score L3 " + state.name(),
                    // none()
                    parallel(
                        run(() -> swerve.drive(0, 0, 0)),
                        sequence(
                            waitSeconds(0.8),
                            robot.setStateCommand(AUTO_HOLD),
                            robot.setStateCommand(RPL3)
                        )
                    ).withDeadline(robot.alignScoreCoral(FieldConstants.allianceFlipRotationally(state.getPose2d()), () -> false).
                        andThen(() -> robot.autoScore()).
                        andThen(waitUntil(() -> robot.stateEquals(NEUTRAL))))
                );
            } else if (state.name().length() == 8) {
                NamedCommands.registerCommand(
                    "Intake " + state.name(),
                    parallel(
                        run(() -> swerve.drive(0, 0, 0))
                    ).withDeadline(
                        robot.alignAlgaeIntake(FieldConstants.allianceFlipRotationally(state.getPose2d()))
                    )
                );
            } else {
                NamedCommands.registerCommand(
                    "Align " + state.name(),
                    run(() -> swerve.drive(0, 0, 0))
                        .withDeadline(robot.alignCoralIntake().withTimeout(1.8)) // swerve.autoAlign(state.getPose2d())
                );
            }
        }

        NamedCommands.registerCommand(
            "Score ALGAE",
            parallel(
                run(() -> swerve.drive(0, 0, 0))
            ).withDeadline(
                robot.alignAlgaeScore(allianceFlipRotationally(new Pose2d(7.7, 5.4, Rotation2d.kZero)))
                    .andThen(waitUntil(() -> robot.stateEquals(NEUTRAL)))
            )
        );

        NamedCommands.registerCommand(
            "Reset RIGHT_BARGE",
            runOnce(() -> swerve.resetOdometryNoGyro(FieldConstants.allianceFlipRotationally(new Pose2d(7.17, 2.55, Rotation2d.kPi))))
        );

        NamedCommands.registerCommand(
            "Reset LEFT_BARGE",
            runOnce(() -> swerve.resetOdometryNoGyro(FieldConstants.allianceFlipRotationally(new Pose2d(7.17, 5.502, Rotation2d.kPi))))
        );

        NamedCommands.registerCommand(
            "Reset MID",
            runOnce(() -> swerve.resetOdometryNoGyro(FieldConstants.allianceFlipRotationally(new Pose2d(7.17, 3.995, Rotation2d.kPi))))
        );

        NamedCommands.registerCommand("Score L2", sequence(
            robot.setStateCommand(RPL2),
            waitUntil(()-> ElevatorMechanism.getInstance().atSetpoint()),
            robot.setStateCommand(RSL2),
            waitSeconds(.25)
        ));

        NamedCommands.registerCommand("Neutral", sequence(
            waitSeconds(0.25),
            robot.setStateCommand(NEUTRAL)
        ));

        NamedCommands.registerCommand("L4", sequence(
            waitSeconds(0.25),
            robot.setStateCommand(RPL4)
        ));
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
        String selectedAutoName = null;
        // String selectedAutoName = NarwhalDashboard.getInstance().getSelectedAuto(); //NarwhalDashboard.getInstance().getSelectedAuto();
        // String hardcode = "MID_3pc_H_auto";
        String hardcode = "LB_3pc_ILK_auto";
        // String hardcode = "MID_1pc_H_auto"; 
        // String hardcode = "Left_Leave_Backwards";
        
         
        Command autoCommand;
        if (selectedAutoName == null) {
            selectedAutoName = hardcode;
        }
        else if (selectedAutoName.equals("default")) {
            defaultAuto();
        }
        autoCommand = autoMap.get(selectedAutoName);

        Log.info("AUTO_SELECTED", selectedAutoName);
        return autoCommand;
    }

    private Command defaultAuto(){
        return none();
    }

    private Command reset() {
        return runOnce(()->swerve.resetGyro(Robot.getAlliance() == Alliance.Red ? 0 : 180));
    }

    public Command pathToPose(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                3.0, 3.0,
                Units.degreesToRadians(171), Units.degreesToRadians(360));

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0
        );
    }

    public Command pathToNearestPose(List<Pose2d> poses) {
        return pathToPose(swerve.getPose().nearest(poses));
    }
}
