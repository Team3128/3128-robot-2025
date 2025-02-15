package frc.team3128.commands;

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


public class CmdAlignReef extends Command {
    
    private Swerve swerve;
    private Led led;

    private double tolerance = 0.05;

    public CmdAlignReef() {
        swerve = Swerve.getInstance();
        led = Led.getInstance();
        addRequirements(led);
    }

    @Override
    public void initialize() {
        swerve.snapToReef();
    }

    @Override
    public void execute() {
        Translation2d pose = swerve.getPose().getTranslation();
        Translation2d snappedReef = swerve.snappedReef.getTranslation();
        
        double cross = Vector.cross(VecBuilder.fill(snappedReef.getX(), snappedReef.getY(), 0), VecBuilder.fill(pose.getX(), pose.getY(), 0)).get(2);
        double error = pose.minus(snappedReef).getNorm();

        if(error < tolerance) {
            led.setLedColor(LedStates.GREEN, 1);
            return;
        }

        if(cross > 0) {
            //left led
            led.setLedColor(LedStates.PURPLE, tolerance/error);
        }
        else if(cross < 0) {
            //right led
            led.setLedColor(LedStates.ORANGE, tolerance/error);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.snappedReef = null;
    }

    @Override
    public boolean isFinished() {
        return !RobotManager.getInstance().stateEquals(RobotStates.defaultElevatorStates.appendAll(RobotStates.exclusiveElevatorStates).asJava());
    }


}
