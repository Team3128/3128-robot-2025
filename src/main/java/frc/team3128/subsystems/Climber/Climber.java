package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.team3128.subsystems.Climber.ClimberStates.*;

import java.util.List;
import java.util.function.Function;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    private static Climber instance;

    public WinchMechanism winch = new WinchMechanism();
    public RollerMechanism roller = new RollerMechanism();

    private static TransitionMap<ClimberStates> transitionMap = new TransitionMap<ClimberStates>(ClimberStates.class);

    private Function<Neutral, Command> setNeutralMode = mode -> runOnce(() -> getMechanisms().forEach(subsystem -> subsystem.setNeutralMode(mode)));
    private Function<ClimberStates, Command> defaultTransitioner = state -> {
        return Commands.sequence(
            setNeutralMode.apply(Neutral.BRAKE),
            roller.runCommand(state.getHasRoller() ? 0.5 : 0),
            winch.pidTo(state.getAngle())
            // runOnce(()->winch.lockServo.setPosition(state.getHasClaw() ? 1 : 0)),
            // runOnce(()->winch.winchServo.setPosition(state.getHasWinch() ? 1 : 0))
        );
    };

    public Climber() {
        super(ClimberStates.class, transitionMap, UNDEFINED);
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addUndefinedState(
            UNDEFINED, 
            NEUTRAL, 
            stopCommand().andThen(() -> setNeutralMode(Neutral.COAST)),
            defaultTransitioner.apply(NEUTRAL).beforeStarting(() -> setNeutralMode(Neutral.BRAKE))
        );

        transitionMap.addCommutativeTransition(List.of(NEUTRAL, CLIMB_PRIME, CLIMB), defaultTransitioner);
	}
}
