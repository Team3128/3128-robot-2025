package frc.team3128.subsystems.Leds;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
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

    private Function<LedsStates, Command> defaultTransitioner = state -> {
        if (state.getIsAnimation()) {
            return Commands.runOnce(() -> LedsMechanism.getInstance().setAnimation(state.getAnimation()));
        } else {
            return Commands.runOnce(() -> LedsMechanism.getInstance().setColor(state.getColor()));
        }
    };

    public static synchronized Leds getInstance() {
        if (instance == null)
            instance = new Leds();
        return instance;
    }

    private Leds() {
        super(LedsStates.class, transitionMap, LedsStates.DISABLED);
    }

    @Override
    public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(LedsStates.values()), defaultTransitioner);
        transitionMap.addConvergingTransition(LedsStates.NEUTRAL, run(
            () -> LedsMechanism.getInstance().setColor(LedsStates.NEUTRAL.getColor(), (int) (LedsConstants.MAX_HEIGHT * Math.max(1.0, 0.03 / getClosestDist())))
        ).until(() -> this.currentState != LedsStates.NEUTRAL));
    }

    final List<Pose2d> coralSetpoints = FieldStates.reefPoses.asJava();
    Supplier<Pose2d> pose = ()-> Swerve.getInstance().nearestPose2d(allianceFlip(coralSetpoints));

    private double getClosestDist() {
        return Swerve.getInstance().getPose().minus(pose.get()).getTranslation().getNorm();
    }
}
