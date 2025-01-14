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
    private Function<Neutral, Command> setNeutralMode = mode -> runOnce(() -> setNeutralMode(mode));
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
        // Idle on
		transitionMap.addConvergingTransition(
            IDLE,
            sequence(
                pivot.stop(),
                roller.stop(),
                runOnce(()->pivot.setNeutralMode(COAST)),
                runOnce(()->roller.setNeutralMode(COAST))
            )
        );

        // Idle off, Intake released, Outtake released, Processor complete, Climb Complete
        transitionMap.addConvergingTransitions(
            transitioner.apply(NEUTRAL),
            NEUTRAL,
            IDLE, INTAKE, EJECT_OUTTAKE, PROCESSOR_OUTTAKE, CLIMB_LOCKED
        );

        // Intake Toggled1, Intake Toggled1*
        transitionMap.addConvergingTransitions(
            transitioner.apply(INTAKE), 
            INTAKE, 
            NEUTRAL, PROCESSOR_OUTTAKE
        );

        transitionMap.addConvergingTransitions(
            transitioner.apply(EJECT_OUTTAKE), 
            EJECT_OUTTAKE, 
            NEUTRAL, PROCESSOR_OUTTAKE
        );

        // Processor Toggled1
        transitionMap.addTransition(
            NEUTRAL, 
            PROCESSOR_PRIME, 
            transitioner.apply(PROCESSOR_PRIME)
        );

        // Processor Toggled2
        transitionMap.addTransition(
            PROCESSOR_PRIME, 
            PROCESSOR_OUTTAKE, 
            transitioner.apply(PROCESSOR_OUTTAKE)
        );

        // Climb Held
        transitionMap.addTransition(
            NEUTRAL, 
            CLIMB_PRIME, 
            transitioner.apply(CLIMB_PRIME)
        );

        // Climb Released
        transitionMap.addTransition(
            CLIMB_PRIME, 
            CLIMB_LOCKED, 
            transitioner.apply(CLIMB_LOCKED)
        );
	}
}
