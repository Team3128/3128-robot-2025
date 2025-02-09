// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.Optional;

import common.core.misc.NAR_Robot;
import common.core.swerve.SwerveModule;
import common.hardware.camera.Camera;
import common.utility.Log;
import static common.utility.Log.Type.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Swerve;
// import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;
import static frc.team3128.Constants.SwerveConstants.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends NAR_Robot {

    public static Alliance alliance;

    public static Alliance getAlliance() {
        if (alliance == null) {
            Optional<Alliance> DSalliance = DriverStation.getAlliance();
            if (DSalliance.isPresent()) alliance = DSalliance.get();
        }
        return alliance;
    }

    public static Robot instance;

    public static RobotContainer m_robotContainer = new RobotContainer();
    // public static AutoPrograms autoPrograms;

    public static synchronized Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    @Override
    public void robotInit(){
        Camera.enableAll();
        m_robotContainer.initDashboard();
        LiveWindow.disableAllTelemetry();
        // Log.logDebug = true;
        Log.Type.enable(STATE_MACHINE_PRIMARY, STATE_MACHINE_SECONDARY, MECHANISM, MOTOR);

    }

    @Override
    public void driverStationConnected() {
        Log.info("State", "DS Connected");
        Log.info("Alliance", getAlliance().toString());
    }

    @Override
    public void robotPeriodic(){
        CommandScheduler.getInstance().run();
        Camera.updateAll();
    }

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().cancelAll();
        Command m_autonomousCommand = AutoPrograms.getInstance().getAutonomousCommand();
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        Camera.updateAll();
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        RobotManager.getInstance().setState(RobotStates.NEUTRAL);
        // for (SwerveModule module : Swerve.getInstance().getModules()) {
        //     module.setBrakeMode(true);
        // }
        Log.info("Robot MOI", ""+ROBOT_MOI);
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
        Camera.updateAll();
    }

    // @Override
    // public void simulationInit() {
        
    // }

    // @Override
    // public void simulationPeriodic() {
    //     CommandScheduler.getInstance().run();
    // }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        Swerve.getInstance().setBrakeMode(false);
        RobotManager.getInstance().stop();
    }

    @Override
    public void disabledExit() {
        Swerve.getInstance().setBrakeMode(true);
    }
    
    // @Override
    // public void disabledPeriodic() {
    //     CommandScheduler.getInstance().run();
    // }
}
