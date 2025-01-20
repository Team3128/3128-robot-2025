package frc.team3128.subsystems.Elevator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.Transition;
import common.core.fsm.TransitionMap;
import common.utility.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Elevator.ElevatorStates.*;

public class Elevator extends FSMSubsystemBase<ElevatorStates> {
    private static Elevator instance;

    protected ElevatorMechanism elevator;
    private static TransitionMap<ElevatorStates> transitionMap = new TransitionMap<ElevatorStates>(ElevatorStates.class);

    public Elevator() {
        super(ElevatorStates.class, transitionMap, NEUTRAL);
        elevator = new ElevatorMechanism();
        addSubsystem(elevator);
        // registerTransitions();
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {

        //ALL STATES -> IDLE
		transitionMap.addConvergingTransition(IDLE, sequence(
                elevator.stop(),
                runOnce(()-> setNeutralMode(COAST))
        ));

        //NEUTRAL, SOURCE, L1, L2, L3, L4 are able to transition between each other
        transitionMap.applyCommutativeFunction(
            state -> {
                return sequence(
                    // runOnce(()-> setNeutralMode(BRAKE)),
                    // elevator.pidTo(state.getSetpoint())
                    elevator.run(state.getSetpoint())
                );
            }, 
            functionalStates
        );

        //IDLE -> NEUTRAL
        transitionMap.addTransition(
            IDLE, 
            NEUTRAL, 
            sequence(
                    runOnce(()-> elevator.setNeutralMode(BRAKE)),
                    // elevator.pidTo(NEUTRAL.getSetpoint())
                    elevator.run(0)
            )
        );
	}
}
