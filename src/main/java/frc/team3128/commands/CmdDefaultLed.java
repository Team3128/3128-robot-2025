package frc.team3128.commands;

import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Led.Led;
import frc.team3128.subsystems.Led.LedStates;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;

import static frc.team3128.Constants.FieldConstants.FieldStates.*;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.LedConstants.*;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import common.hardware.camera.Camera;


public class CmdDefaultLed extends Command {
    
    private Swerve swerve;
    private Led led;

    public CmdDefaultLed() {
        swerve = Swerve.getInstance();
        led = Led.getInstance();
        addRequirements(led);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Translation2d robotPos = swerve.getTranslation();
        // Translation2d snappedReef = swerve.getClosestReef().getTranslation();
        FieldStates closest = FieldStates.A;
        for(FieldStates state : FieldStates.values()) {
            if(robotPos.getDistance(state.getTranslation2d()) < robotPos.getDistance(closest.getTranslation2d())) {
                closest = state;
            }
        }
        
        double distance = robotPos.getDistance(closest.getTranslation2d());
        
        if(distance > 2.5) {
            led.setLedColor(LedStates.DEFAULT);
        }
        else{
            if(Camera.seesTag()){
                led.candle.animate(new TwinkleAnimation(closest.getLedState().r, closest.getLedState().g, closest.getLedState().b), 0);
            }
            else{
                led.setLedColor(closest.getLedState());
            }
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return !RobotManager.getInstance().stateEquals(RobotStates.defaultElevatorStates.appendAll(RobotStates.neutralStates).asJava());
    }


}