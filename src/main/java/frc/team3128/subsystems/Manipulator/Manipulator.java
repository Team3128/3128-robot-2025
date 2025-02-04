package frc.team3128.subsystems.Manipulator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Manipulator.ManipulatorStates.*;

import java.util.function.Function;

public class Manipulator extends FSMSubsystemBase<ManipulatorStates> {
    private static Manipulator instance;

    // public RollerMechanism roller;
    private static TransitionMap<ManipulatorStates> transitionMap = new TransitionMap<ManipulatorStates>(ManipulatorStates.class);
    private Function<ManipulatorStates, Command> defaultTransitioner = state -> {return runVoltsCommand(state.getVolts());};

    public Manipulator() {
        super(ManipulatorStates.class, transitionMap, NEUTRAL);
        // roller = RollerMechanism.getInstance();
        // addMechanisms(roller);
        registerTransitions();
    }

    public static synchronized Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

	@Override
	public void registerTransitions() {

        //ALL STATES -> IDLE
		transitionMap.addConvergingTransition(
            IDLE, 
            sequence(
                runCommand(IDLE.getVolts()),
                runOnce(()-> setNeutralMode(COAST))
            )
        );

        //NEUTRAL, IN, OUT are able to transition between each other
        transitionMap.addCommutativeTransition(functionalStates.asJava(), defaultTransitioner);

        //IDLE -> NEUTRAL
        transitionMap.addTransition(
            IDLE, 
            NEUTRAL, 
            sequence(
                runCommand(NEUTRAL.getVolts()),
                runOnce(()-> setNeutralMode(BRAKE))
            )
        );
	}
}
