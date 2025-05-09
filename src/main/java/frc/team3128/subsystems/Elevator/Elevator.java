package frc.team3128.subsystems.Elevator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.RobotContainer;
import frc.team3128.subsystems.Swerve;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static frc.team3128.subsystems.Elevator.ElevatorStates.*;

import java.util.function.Function;

public class Elevator extends FSMSubsystemBase<ElevatorStates> {
    private static Elevator instance;

    protected ElevatorMechanism elevator;
    private static TransitionMap<ElevatorStates> transitionMap = new TransitionMap<ElevatorStates>(ElevatorStates.class);
    private Function<ElevatorStates, Command> defaultTransitioner = state -> {return elevator.pidTo(RobotContainer.shouldRam.getAsBoolean() ? state.getSetpointRam() : state.getSetpointRamless());};

    public Elevator() {
        super(ElevatorStates.class, transitionMap, NEUTRAL);
        elevator = ElevatorMechanism.getInstance();
        addMechanisms(elevator);
        registerTransitions();
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addCommutativeTransition(functionalStates.asJava(), defaultTransitioner);
	}
}
