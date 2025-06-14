package frc.team3128.subsystems.Leds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.Constants.LedsConstants;
import frc.team3128.Constants.FieldConstants.FieldStates;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;
import static frc.team3128.Constants.FieldConstants.*;

public class Leds extends FSMSubsystemBase<LedsStates> {

    private static Leds instance;
    private static TransitionMap<LedsStates> transitionMap = new TransitionMap<LedsStates>(LedsStates.class);
    private int h = 0;

    private Function<LedsStates, Command> defaultTransitioner = state -> {
        if (state.getIsAnimation()) {
            return runOnce(() -> LedsMechanism.getInstance().setAnimation(state.getAnimation())).ignoringDisable(true);
        } else {
            return runOnce(() -> LedsMechanism.getInstance().setColor(state.getColor())).ignoringDisable(true);
        }
    };

    public static synchronized Leds getInstance() {
        if (instance == null)
            instance = new Leds();
        return instance;
    }

    private Leds() {
        super(LedsStates.class, transitionMap, LedsStates.DISABLED);
        LedsMechanism.getInstance().setColor(LedsStates.DISABLED.getColor());
        addMechanisms(LedsMechanism.getInstance());
        registerTransitions();
        initShuffleboard();
    }

    @Override
    public void periodic() {
        if (getState() == LedsStates.NEUTRAL) {
            h = (int) (LedsConstants.MAX_HEIGHT * Math.min(1.0, 0.03 / getClosestDist()));
            LedsMechanism.getInstance().setColor(LedsStates.NEUTRAL.getColor(), h);
        }
    }

    @Override
    public void initShuffleboard() {
        super.initShuffleboard();
        NAR_Shuffleboard.addData("Leds", "Height", ()-> h);
        NAR_Shuffleboard.addData("Leds", "Closest Dist", () -> getClosestDist());
    }

    @Override
    public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(LedsStates.values()), defaultTransitioner);
        transitionMap.addConvergingTransition(LedsStates.NEUTRAL, run(
            () -> {h = (int) (LedsConstants.MAX_HEIGHT * Math.max(1.0, 0.03 / getClosestDist())); LedsMechanism.getInstance().setColor(LedsStates.NEUTRAL.getColor(), h);}
        ).until(() -> getState() != LedsStates.NEUTRAL).ignoringDisable(true));
    }

    final List<Pose2d> coralSetpoints = FieldStates.reefPoses.asJava();
    Supplier<Pose2d> pose = ()-> Swerve.getInstance().nearestPose2d(allianceFlip(coralSetpoints));

    private double getClosestDist() {
        return Swerve.getInstance().getPose().minus(pose.get()).getTranslation().getNorm();
    }
}
