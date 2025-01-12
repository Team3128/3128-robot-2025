package frc.team3128.subsystems.Elevator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Elevator.ElevatorStates.*;

public class Elevator extends FSMSubsystemBase<ElevatorStates> {
    private static Elevator instance;

    private ElevatorMechanism elevator;
    private static TransitionMap<ElevatorStates> transitionMap;

    private Elevator() {
        super(ElevatorStates.class, transitionMap, IDLE);
        elevator = new ElevatorMechanism();
        addSubsystem(elevator);
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
		transitionMap.addConvergingTransition(IDLE, sequence(
                elevator.stop(),
                runOnce(()-> setNeutralMode(COAST))
        ));

        transitionMap.applyCommutativeFunction(
            state -> {
                return sequence(
                    runOnce(()-> setNeutralMode(BRAKE)),
                    elevator.pidTo(state.getSetpoint())
                );
            }, 
            funtionalStates
        );

        transitionMap.addTransition(
            IDLE, 
            NEUTRAL, 
            sequence(
                    runOnce(()-> elevator.setNeutralMode(BRAKE)),
                    elevator.pidTo(NEUTRAL.getSetpoint())
            )
        );

        transitionMap.addDivergingTransition(IDLE);
        transitionMap.addTransition(IDLE, NEUTRAL, runOnce(()-> elevator.setNeutralMode(BRAKE)));

	}
}
