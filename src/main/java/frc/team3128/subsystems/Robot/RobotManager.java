package frc.team3128.subsystems.Robot;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Manipulator.Manipulator;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.util.function.BooleanSupplier;

public class RobotManager extends FSMSubsystemBase<RobotStates> {
    private static RobotManager instance;
    private static Elevator elevator;
    private static Manipulator manipulator;
    private static Intake intake;
    private static Climber climber;

    private static TransitionMap<RobotStates> transitionMap;

    private RobotManager() {
        super(RobotStates.class, transitionMap, IDLE);
        registerTransitions();

        elevator = Elevator.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
    }

    public static synchronized RobotManager getInstance() {
        if (instance == null) {
            instance = new RobotManager();
        }

        return instance;
    }

    public Command updateSubsystemStates(RobotStates nextState) {
        return sequence(
            elevator.setStateCommand(nextState.getElevatorState()),
            manipulator.setStateCommand(nextState.getManipulatorState()),
            intake.setStateCommand(nextState.getIntakeState()),
            climber.setStateCommand(nextState.getClimberState())
        );
    }

	@Override
	public void registerTransitions() {
		
	}

    public Command getCoralState(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        boolean initialManipulatorState = Manipulator.getInstance().hasObjectPresent();
        return either(
            setStateCommand(exclusiveState)
            .until(()-> !initialManipulatorState)
            .andThen(setStateCommand(NEUTRAL)),
            setStateCommand(defaultState),
            condition
        );
    }

    public Command getCoralState(RobotStates defaultState, RobotStates exclusiveState) {
        return getCoralState(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command getAlgeaState(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        boolean initialIntakeState = Intake.getInstance().hasObjectPresent();
        return either(
            setStateCommand(exclusiveState)
            .until(()-> !initialIntakeState)
            .andThen(setStateCommand(NEUTRAL)),
            setStateCommand(defaultState),
            condition
        );
    }

    public Command getAlgeaState(RobotStates defaultState, RobotStates exclusiveState) {
        return getAlgeaState(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command getAlgeaState(RobotStates state, BooleanSupplier condition){
        return either(
            setStateCommand(NEUTRAL),
            setStateCommand(state),
            condition
        );
    }

    public Command getAlgeaState(RobotStates state){
        return getAlgeaState(state, ()-> stateEquals(state));
    }

    public Command getClimbState() {
        return either(
            setStateCommand(CLIMB_LOCK), 
            setStateCommand(CLIMB_PRIME), 
            ()-> stateEquals(CLIMB_PRIME)
        );
    }
}
