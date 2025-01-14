package frc.team3128.subsystems.Intake;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.hardware.motorcontroller.NAR_Motor.Neutral;

import static frc.team3128.subsystems.Intake.IntakeStates.*;
import java.util.function.Function;

public class Intake extends FSMSubsystemBase<IntakeStates> {
    
    private static Intake instance;

    protected PivotMechanism pivot;
    protected RollerMechanism roller;
    private static TransitionMap<IntakeStates> transitionMap = new TransitionMap<IntakeStates>(IntakeStates.class);
    private Function<Neutral, Command> setNeutralMode = mode -> runOnce(() -> getSubsystems().forEach(subsystem -> subsystem.setNeutralMode(mode)));
    private Function<IntakeStates, Command> transitioner = state -> {
        return sequence(
            setNeutralMode.apply(BRAKE),
            pivot.pidTo(state.getAngle()),
            roller.run(state.getPower())
        );
    };

    public Intake() {
        super(IntakeStates.class, transitionMap, IDLE);
        pivot = new PivotMechanism();
        roller = new RollerMechanism();
        addSubsystem(pivot, roller);
        registerTransitions();

        System.out.println(transitionMap);
    }

    public static synchronized Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    public boolean hasObjectPresent() {
        return roller.hasObjectPresent();
    }

	@Override
	public void registerTransitions() {
        //ALL STATES -> IDLE
		transitionMap.addConvergingTransition(
            IDLE,
            sequence(
                pivot.stop(),
                roller.stop(),
                runOnce(()->pivot.setNeutralMode(COAST)),
                runOnce(()->roller.setNeutralMode(COAST))
            )
        );

        //IDLE, INTAKE, EJECT_OUTTAKE, PROCESSOR_PRIME, PROCESSOR_OUTTAKE, CLIMB_LOCKED, CLIMB -> NEUTRAL
        transitionMap.addConvergingTransitions(
            transitioner.apply(NEUTRAL),
            NEUTRAL,
            IDLE, INTAKE, EJECT_OUTTAKE, PROCESSOR_PRIME, PROCESSOR_OUTTAKE, CLIMB_LOCKED, CLIMB
        );

        //NEUTRAL, PROCESSOR_OUTTAKE -> INTAKE
        transitionMap.addConvergingTransitions(
            transitioner.apply(INTAKE), 
            INTAKE, 
            NEUTRAL, PROCESSOR_OUTTAKE
        );

        // NEUTRAL, INTAKE -> EJECT_OUTTAKE
        transitionMap.addConvergingTransitions(
            transitioner.apply(EJECT_OUTTAKE), 
            EJECT_OUTTAKE, 
            NEUTRAL, PROCESSOR_OUTTAKE
        );

        // NEUTRAL -> PROCESSOR_PRIME
        transitionMap.addTransition(
            NEUTRAL, 
            PROCESSOR_PRIME, 
            transitioner.apply(PROCESSOR_PRIME)
        );

        // PROCESSOR_PRIME -> PROCESSOR_OUTTAKE
        transitionMap.addTransition(
            PROCESSOR_PRIME, 
            PROCESSOR_OUTTAKE, 
            transitioner.apply(PROCESSOR_OUTTAKE)
        );

        // NEUTRAL -> CLIMB_PRIME
        transitionMap.addTransition(
            NEUTRAL, 
            CLIMB_PRIME, 
            transitioner.apply(CLIMB_PRIME)
        );

        // CLIMB_PRIME -> CLIMB_LOCKED
        transitionMap.addTransition(
            CLIMB_PRIME, 
            CLIMB_LOCKED, 
            transitioner.apply(CLIMB_LOCKED)
        );

        // CLIMB_LOCKED -> CLIMB
        transitionMap.addTransition(
            CLIMB_LOCKED, 
            CLIMB, 
            transitioner.apply(CLIMB)
        );
        
	}
}
