package frc.team3128;


import common.hardware.input.NAR_XboxController;
import common.hardware.input.NAR_XboxController.XboxButton;
import common.hardware.limelight.Limelight;
import common.hardware.motorcontroller.NAR_CANSpark;
import common.hardware.motorcontroller.NAR_TalonFX;

import static common.hardware.input.NAR_XboxController.XboxButton.*;

import common.hardware.input.NAR_ButtonBoard;

import common.hardware.camera.Camera;
import common.utility.narwhaldashboard.NarwhalDashboard;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static edu.wpi.first.wpilibj2.command.Commands.*;
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
        
        swerveDriveCommand = swerve.getDriveCommand(controller::getLeftX,controller::getLeftY, controller::getRightX);

        //uncomment line below to enable driving
        CommandScheduler.getInstance().setDefaultCommand(swerve, swerveDriveCommand);
        
        DriverStation.silenceJoystickConnectionWarning(true);
        initCameras();
        configureButtonBindings();
        initDashboard();
    }   
    private void configureButtonBindings() {
        controller.getButton(XboxButton.kB).onTrue(runOnce(()->swerve.resetGyro(0)));
        controller.getButton(XboxButton.kA).onTrue(runOnce(()->swerve.resetEncoders()));
        controller.getButton(XboxButton.kY).onTrue(swerve.characterize(1, 1).beforeStarting(runOnce(()->swerve.zeroLock())));
    }
    public double inchesToMeters(double inches){
        return Distance.ofRelativeUnits(inches, Units.Inches).in(Units.Meters);
    }

    public double degreesToRadians(double degrees){
        return Angle.ofRelativeUnits(degrees, Units.Degrees).in(Units.Radians);
    }

    private void configureButtonBindings() {
        controller.getButton(XboxButton.kA).onTrue(swerve.characterize(1, 1).beforeStarting(runOnce(()->swerve.oLock())));
        controller.getButton(XboxButton.kB).onTrue(runOnce(()->swerve.resetGyro(0)));
        controller.getButton(XboxButton.kY).onTrue(runOnce(()->swerve.resetEncoders()));

        buttonPad.getButton(1).onTrue(runOnce(()-> Arrays.asList(swerve.getModules()).forEach(module -> module.setBrakeMode(false))))
        .onFalse(runOnce(()-> Arrays.asList(swerve.getModules()).forEach(module -> module.setBrakeMode(true))));
        buttonPad.getButton(2).onTrue(runOnce(()-> swerve.oLock()));
        buttonPad.getButton(3).onTrue(runOnce(()-> swerve.zeroLock()));
    }
    public void initCameras() { 
        List<AprilTag> tags = new ArrayList<AprilTag>();
        tags.add(new AprilTag(1, new Pose3d(inchesToMeters(657.37), inchesToMeters(25.80), inchesToMeters(58.50), new Rotation3d(Math.toRadians(126), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(2, new Pose3d(inchesToMeters(657.37), inchesToMeters(291.20), inchesToMeters(58.50), new Rotation3d(Math.toRadians(234), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(3, new Pose3d(inchesToMeters(455.15), inchesToMeters(317.15), inchesToMeters(51.25), new Rotation3d(Math.toRadians(270), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(4, new Pose3d(inchesToMeters(365.20), inchesToMeters(241.64), inchesToMeters(73.54), new Rotation3d(Math.toRadians(0), 0, Math.toRadians(30)))));
        tags.add(new AprilTag(5, new Pose3d(inchesToMeters(265.20), inchesToMeters(75.39), inchesToMeters(73.54), new Rotation3d(Math.toRadians(0), 0, Math.toRadians(30)))));
        tags.add(new AprilTag(6, new Pose3d(inchesToMeters(530.49), inchesToMeters(130.17), inchesToMeters(12.13), new Rotation3d(Math.toRadians(300), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(7, new Pose3d(inchesToMeters(546.87), inchesToMeters(158.50), inchesToMeters(12.13), new Rotation3d(Math.toRadians(0), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(8, new Pose3d(inchesToMeters(530.49), inchesToMeters(186.83), inchesToMeters(12.13), new Rotation3d(Math.toRadians(60), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(9, new Pose3d(inchesToMeters(497.77), inchesToMeters(186.83), inchesToMeters(12.13), new Rotation3d(Math.toRadians(120), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(10, new Pose3d(inchesToMeters(481.39), inchesToMeters(158.50), inchesToMeters(12.13), new Rotation3d(Math.toRadians(180), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(11, new Pose3d(inchesToMeters(497.77), inchesToMeters(130.17), inchesToMeters(12.13), new Rotation3d(Math.toRadians(240), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(12, new Pose3d(inchesToMeters(33.51), inchesToMeters(25.80), inchesToMeters(58.50), new Rotation3d(Math.toRadians(54), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(13, new Pose3d(inchesToMeters(33.51), inchesToMeters(291.20), inchesToMeters(58.50), new Rotation3d(Math.toRadians(306), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(14, new Pose3d(inchesToMeters(325.68), inchesToMeters(241.64), inchesToMeters(73.54), new Rotation3d(Math.toRadians(180), 0, Math.toRadians(30)))));
        tags.add(new AprilTag(15, new Pose3d(inchesToMeters(325.68), inchesToMeters(75.39), inchesToMeters(73.54), new Rotation3d(Math.toRadians(180), 0, Math.toRadians(30)))));
        tags.add(new AprilTag(16, new Pose3d(inchesToMeters(235.73), inchesToMeters(-0.15), inchesToMeters(51.25), new Rotation3d(Math.toRadians(90), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(17, new Pose3d(inchesToMeters(160.39), inchesToMeters(130.17), inchesToMeters(12.13), new Rotation3d(Math.toRadians(240), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(18, new Pose3d(inchesToMeters(144.00), inchesToMeters(158.50), inchesToMeters(12.13), new Rotation3d(Math.toRadians(180), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(19, new Pose3d(inchesToMeters(160.39), inchesToMeters(186.83), inchesToMeters(12.13), new Rotation3d(Math.toRadians(120), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(20, new Pose3d(inchesToMeters(193.10), inchesToMeters(186.83), inchesToMeters(12.13), new Rotation3d(Math.toRadians(60), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(21, new Pose3d(inchesToMeters(209.49), inchesToMeters(158.58), inchesToMeters(12.13), new Rotation3d(Math.toRadians(0), 0, Math.toRadians(0)))));
        tags.add(new AprilTag(22, new Pose3d(inchesToMeters(193.10), inchesToMeters(130.17), inchesToMeters(12.13), new Rotation3d(Math.toRadians(300), 0, Math.toRadians(0)))));
        
        AprilTagFieldLayout tagLayout = new AprilTagFieldLayout(tags, 17.548, 8.052);

        Camera.setResources(()->swerve.getYaw(), (pose, time) -> swerve.addVisionMeasurement(pose, time), tagLayout, () -> swerve.getPose());
         Camera.setThresholds(20,  10);
         if(Robot.isReal()){
            Camera frontRightCamera = new Camera("FRONT_RIGHT", inchesToMeters(10.055), -inchesToMeters(9.79), degreesToRadians(30), degreesToRadians(-28.125), 0);
            // Camera frontLeftCamera = new Camera("FRONT_LEFT", inchesToMeters(10.055), -inchesToMeters(9.79), degreesToRadians(-30), degreesToRadians(-28.125), 0);

            Camera frontLeftCamera = new Camera("FRONT_LEFT", inchesToMeters(13.5), inchesToMeters(0), degreesToRadians(0), degreesToRadians(0), 0);
            Camera backRightCamera = new Camera("BACK_RIGHT", -inchesToMeters(10.055), -inchesToMeters(9.79),  degreesToRadians(150), degreesToRadians(-28.125), 0);
            Camera backLeftCamera = new Camera("BACK_LEFT", -inchesToMeters(10.055), inchesToMeters(9.79), degreesToRadians(-150), degreesToRadians(-28.125), 0);
         }  
    }

    public void initDashboard() {
        dashboard = NarwhalDashboard.getInstance();
        dashboard.addUpdate("robotX", ()-> swerve.getPose().getX());
        dashboard.addUpdate("robotY", ()-> swerve.getPose().getY());
        dashboard.addUpdate("robotYaw", ()-> swerve.getPose().getRotation().getDegrees());
    }
}
