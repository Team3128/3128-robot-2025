package frc.team3128.subsystems.Elevator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Elevator.ElevatorStates.*;

import java.util.function.Function;

public class Elevator extends FSMSubsystemBase<ElevatorStates> {
    private static Elevator instance;

    protected ElevatorMechanism elevator;
    private static TransitionMap<ElevatorStates> transitionMap = new TransitionMap<ElevatorStates>(ElevatorStates.class);
    private Function<ElevatorStates, Command> defaultTransitioner = state -> {return elevator.pidTo(state.getSetpoint());};
    private Function<ElevatorStates, Command> idleTransitioner = state -> {return stopCommand().andThen(runOnce(()-> setNeutralMode(state.getNeutral())));};

    public Elevator() {
        super(ElevatorStates.class, transitionMap, IDLE);
        elevator = ElevatorMechanism.getInstance();
        addMechanisms(elevator);
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
		transitionMap.addConvergingTransition(IDLE, idleTransitioner);

        transitionMap.addCommutativeTransition(functionalStates.asJava(), defaultTransitioner);

        transitionMap.addTransition(IDLE, NEUTRAL, idleTransitioner);
	}
}
