package frc.team3128.subsystems.Robot;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Manipulator.Manipulator;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class RobotManager extends FSMSubsystemBase<RobotStates> {
    private static RobotManager instance;

    private static TransitionMap<RobotStates> transitionMap;

    private RobotManager() {
        super(RobotStates.class, transitionMap, IDLE);
        // addSubsystem(Elevator.getInstance(), Intake.getInstance(), Manipulator.getInstance(), Climber.getInstance());
    }

    public static synchronized RobotManager getInstance() {
        if (instance == null) {
            instance = new RobotManager();
        }

        return instance;
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
}
