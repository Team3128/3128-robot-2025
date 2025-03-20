package frc.team3128.commands;

import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;
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
import frc.team3128.Constants.FieldConstants.FieldStates;
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
import common.utility.shuffleboard.NAR_Shuffleboard;


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
        // NAR_Shuffleboard.addData("LEDS", "wawawa", ()-> ledSupplier.get().name(), 0, 0);
    }

    @Override
    public void execute() {
        Supplier<LedStates> ledSupplier = ()-> FieldStates.getEnum(Swerve.getInstance().getClosestReef()).getLedState();


        Translation2d robotPos = swerve.getTranslation();
        Translation2d snappedReef = swerve.getClosestReef().getTranslation();

        
        double distance = robotPos.getDistance(snappedReef);
        
        if(distance > 2.5) {
            Led.getInstance().candle.animate(new FireAnimation(BRIGHTNESS, r_SPEED, NUM_LED, SPARKING, COOLING, false, STARTING_ID), 0);
        }
        else{
            if(Camera.seesTag()){
                led.resetAnimationSlot();
                led.candle.animate(new TwinkleAnimation(ledSupplier.get().r, ledSupplier.get().g, ledSupplier.get().b), 0);
            }
            else{
                led.resetAnimationSlot();
                led.setLedColor(ledSupplier.get());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return !(RobotManager.getInstance().stateEquals(RobotStates.NEUTRAL) || RobotManager.getInstance().stateEquals(RobotStates.FULL_NEUTRAL));
    }

}