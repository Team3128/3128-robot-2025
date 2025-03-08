// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3128;

import java.util.Optional;

import common.core.misc.NAR_Robot;
import common.hardware.camera.Camera;
import common.utility.Log;
import static common.utility.Log.Type.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;
import frc.team3128.subsystems.Intake.PivotMechanism;
// import frc.team3128.autonomous.AutoPrograms;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;
import com.pathplanner.lib.commands.PathfindingCommand;

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

    public static AutoPrograms autoPrograms = AutoPrograms.getInstance();
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
        Log.info("Dashboard", "Done");
        LiveWindow.disableAllTelemetry();
        // Log.logDebug = true;
        autoPrograms.initAutoSelector();
        Log.Type.enable(STATE_MACHINE_PRIMARY, STATE_MACHINE_SECONDARY, MECHANISM, MOTOR);
        PathfindingCommand.warmupCommand().schedule();
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
        Camera.enableAll();
        runOnce(()-> {
            Swerve.translationController.disable();
            Swerve.rotationController.disable();
        }).schedule();
        
        Command m_autonomousCommand = autoPrograms.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        else System.out.println("Auto Command is null");
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
        Log.info("A", FieldStates.A.getPose2d().toString());
        Log.info("B", FieldStates.B.getPose2d().toString());
        Log.info("C", FieldStates.C.getPose2d().toString());
        Log.info("D", FieldStates.D.getPose2d().toString());
        Log.info("E", FieldStates.E.getPose2d().toString());
        Log.info("F", FieldStates.F.getPose2d().toString());
        Log.info("G", FieldStates.G.getPose2d().toString());
        Log.info("H", FieldStates.H.getPose2d().toString());
        Log.info("I", FieldStates.I.getPose2d().toString());
        Log.info("J", FieldStates.J.getPose2d().toString());
        Log.info("K", FieldStates.K.getPose2d().toString());
        Log.info("L", FieldStates.L.getPose2d().toString());
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        RobotManager.getInstance().stopCommand().schedule();
        // Log.info("State", RobotManager.getInstance().getState().name());
        RobotManager.getInstance().setStateCommand(RobotStates.NEUTRAL).schedule();
        PivotMechanism.getInstance().stopCommand().schedule();
        // Log.info("State", RobotManager.getInstance().getState().name());
        Camera.enableAll();
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
    public void teleopExit() {
        Log.info("State", RobotManager.getInstance().getState().name());
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        Swerve.getInstance().setBrakeMode(false);
        Swerve.disable();
        RobotManager.getInstance().stopCommand().ignoringDisable(true).schedule();
        Log.info("State", RobotManager.getInstance().getState().name());
    }

    @Override
    public void disabledExit() {
        Swerve.getInstance().setBrakeMode(true);
        RobotManager.getInstance().stop();
        Log.info("State", RobotManager.getInstance().getState().name());
    }
    
    // @Override
    // public void disabledPeriodic() {
    //     CommandScheduler.getInstance().run();
    // }
}
