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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Robot;
import frc.team3128.Constants.LedConstants.Colors;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Led.Led;
import frc.team3128.subsystems.Robot.RobotManager;
import frc.team3128.subsystems.Robot.RobotStates;

import static frc.team3128.Constants.FieldConstants.FieldStates.*;
import static frc.team3128.Constants.FieldConstants.*;
import static frc.team3128.Constants.SwerveConstants.*;


public class CmdAlignReef extends Command {
    
    private static boolean trackRight;
    private Translation2d tracking;
    private Swerve swerve;
    private Translation2d rightShift;
    private Translation2d leftShift;

    private double tolerance = 0.05;
    private Led led;

    public CmdAlignReef() {
        swerve = Swerve.getInstance();
        led = Led.getInstance();
        addRequirements(led);
    }

    @Override
    public void initialize() {
        Pose2d target = swerve.nearestPose2d(reefPoses);
        rightShift = target.getTranslation().plus(reefShift.rotateBy(target.getRotation().plus(Rotation2d.fromDegrees(-90))));
        leftShift = target.getTranslation().plus(reefShift.rotateBy(target.getRotation().plus(Rotation2d.fromDegrees(90))));
        tracking  = swerve.nearestTranslation2d(List.of(rightShift, leftShift));
        trackRight = tracking.equals(rightShift);
    }

    public void toggleTracking() {
        trackRight = !trackRight;
        if(trackRight) tracking = rightShift;
        else tracking = leftShift;
    }

    @Override
    public void execute() {
        Translation2d pose = swerve.getPose().transformBy(new Transform2d(MANIP_OFFSET.rotateBy(swerve.getGyroRotation2d()), new Rotation2d())).getTranslation();
        
        double cross = Vector.cross(VecBuilder.fill(tracking.getX(), tracking.getY(), 0), VecBuilder.fill(pose.getX(), pose.getY(), 0)).get(2);
        double error = pose.minus(tracking).getNorm();

        if(error < tolerance) {
            led.setLedColor(Colors.GREEN, 1);
            return;
        }

        if(cross > 0) {
            //left led
            led.setLedColor(Colors.PURPLE, tolerance/error);
        }
        else if(cross < 0) {
            //right led
            led.setLedColor(Colors.ORANGE, tolerance/error);
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return !RobotManager.getInstance().stateEquals(RobotStates.defaultElevatorStates.appendAll(RobotStates.exclusiveElevatorStates).asJava());
    }


}
