package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.Transition;
import common.core.fsm.TransitionMap;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.subsystems.Intake.IntakeStates;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Climber.ClimberStates.*;

import java.util.function.Function;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    private static Climber instance;

    // public WinchMechanism winch;

    private static TransitionMap<ClimberStates> transitionMap = new TransitionMap<ClimberStates>(ClimberStates.class);

    private Function<Neutral, Command> setNeutralMode = mode -> runOnce(() -> getMechanisms().forEach(subsystem -> subsystem.setNeutralMode(mode)));
    private Function<ClimberStates, Command> transitioner = state -> {
        return sequence(
            setNeutralMode.apply(BRAKE)//,
            // winch.pidTo(state.getAngle())
            // runOnce(()->winch.lockServo.setPosition(state.getHasClaw() ? 1 : 0)),
            // runOnce(()->winch.winchServo.setPosition(state.getHasWinch() ? 1 : 0))
        );
    };

    public Climber() {
        super(ClimberStates.class, transitionMap, NEUTRAL);
        // winch = new WinchMechanism();
        // addSubsystem(winch);
        // registerTransitions();
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {

        //ALL STATES -> IDLE
        transitionMap.addConvergingTransition(
            IDLE,
            sequence(
            runOnce(()-> setNeutralMode(COAST))//,
            // runOnce(()-> winch.stop())
        )
        );

        //IDLE, PRIME, LOCKED -> NEUTRAL
        transitionMap.addConvergingTransitions(
            transitioner.apply(NEUTRAL), 
            NEUTRAL, 
            IDLE, CLIMB_PRIME, CLIMB_LOCKED
        );

        //NEUTRAL -> PRIME
        transitionMap.addTransition(NEUTRAL, CLIMB_PRIME, transitioner.apply(CLIMB_PRIME));

        //PRIME -> LOCKED
        transitionMap.addTransition(CLIMB_PRIME, CLIMB_LOCKED, transitioner.apply(CLIMB_LOCKED));

        //LOCKED -> WINCH
        transitionMap.addTransition(CLIMB_LOCKED, CLIMB_WINCH, transitioner.apply(CLIMB_WINCH));

        //WINCH -> PRIME
        transitionMap.addTransition(CLIMB_LOCKED, CLIMB_PRIME, transitioner.apply(CLIMB_PRIME));
	}
}
