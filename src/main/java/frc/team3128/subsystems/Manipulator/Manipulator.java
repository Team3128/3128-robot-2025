package frc.team3128.subsystems.Manipulator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Manipulator.ManipulatorStates.*;

public class Manipulator extends FSMSubsystemBase<ManipulatorStates> {
    private static Manipulator instance;

    private RollerMechanism manipulator;
    private static TransitionMap<ManipulatorStates> transitionMap;

    private Manipulator() {
        super(ManipulatorStates.class, transitionMap, IDLE);
        manipulator = new RollerMechanism();
        addSubsystem(manipulator);
    }

    public static synchronized Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
		transitionMap.addConvergingTransition(IDLE, sequence(
                manipulator.stop(),
                runOnce(()-> setNeutralMode(COAST))
        ));

        transitionMap.applyCommutativeFunction(
            state -> {
                return sequence(
                    runOnce(()-> setNeutralMode(BRAKE)),
                    manipulator.run(state.getPower())
                );
            }, 
            functionalStates
        );

        transitionMap.addTransition(
            IDLE, 
            NEUTRAL, 
            sequence(
                    runOnce(()-> manipulator.setNeutralMode(BRAKE)),
                    manipulator.run(NEUTRAL.getPower())
            )
        );
	}

    public boolean hasObjectPresent() {
        return false;
    }
}
