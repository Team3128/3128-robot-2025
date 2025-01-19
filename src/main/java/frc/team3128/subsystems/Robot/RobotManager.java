package frc.team3128.subsystems.Robot;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.Transition;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Climber.WinchMechanism;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.Manipulator;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.RobotContainer.printStatus;
import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.util.function.BooleanSupplier;

public class RobotManager extends FSMSubsystemBase<RobotStates> {
    private static RobotManager instance;
    private static Elevator elevator;
    private static Manipulator manipulator;
    private static Intake intake;
    private static Climber climber;

    private static TransitionMap<RobotStates> transitionMap = new TransitionMap<>(RobotStates.class);

    private RobotManager() {
        super(RobotStates.class, transitionMap, NEUTRAL);

        elevator = Elevator.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
        registerTransitions();
    }

    public static synchronized RobotManager getInstance() {
        if (instance == null) {
            instance = new RobotManager();
        }

        return instance;
    }

    public Command updateSubsystemStates(RobotStates nextState) {
        return sequence(
            elevator.setStateCommand(nextState.getElevatorState())
            // manipulator.setStateCommand(nextState.getManipulatorState()),
            // intake.setStateCommand(nextState.getIntakeState()),
            // climber.setStateCommand(nextState.getClimberState())
        );
    }

    public Command getCoralState(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        boolean initialManipulatorState = Manipulator.getInstance().hasObjectPresent();
        return either(
            setStateCommand(exclusiveState)
            // .until(()-> !initialManipulatorState)
            .withTimeout(1)
            .andThen(setStateCommand(NEUTRAL)),
            setStateCommand(defaultState),
            condition
        );
    }

    public Command getCoralState(RobotStates defaultState, RobotStates exclusiveState) {
        return getCoralState(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command getAlgaeState(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        boolean initialIntakeState = Intake.getInstance().hasObjectPresent();
        return either(
            setStateCommand(exclusiveState)
            .until(()-> !initialIntakeState)
            .andThen(setStateCommand(NEUTRAL)),
            setStateCommand(defaultState),
            condition
        );
    }

    public Command getAlgaeState(RobotStates defaultState, RobotStates exclusiveState) {
        return getAlgaeState(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command getAlgaeState(RobotStates state, BooleanSupplier condition){
        return either(
            setStateCommand(NEUTRAL),
            setStateCommand(state),
            condition
        );
    }

    public Command getAlgaeState(RobotStates state){
        return getAlgaeState(state, ()-> stateEquals(state));
    }

    public Command getClimbState() {
        return either(
            setStateCommand(CLIMB_LOCK), 
            setStateCommand(CLIMB_PRIME), 
            ()-> stateEquals(CLIMB_PRIME)
        );
    }

    @Override
	public void registerTransitions() {
        transitionMap.addConvergingTransition(
            IDLE,
            Climber.getInstance().winch.run(0).andThen(print("NEUTRAL->IDLE"))
        );
        // transitionMap.addTransition(
        //     IDLE,
        //     NEUTRAL,
        //     runOnce(() -> WinchMechanism.leader.setPercentOutput(0)).andThen(print("NEUTRAL->IDLE"))
        // );

        transitionMap.applyCommutativeFunction(
            state -> sequence(
                updateSubsystemStates(state),
                print("Transitioning to state: " + state)
            ), 
            reefPrimeStates
        );

        transitionMap.addTransition(
            RPL1,
            RSL1,
            updateSubsystemStates(RSL1)
        );
        transitionMap.addTransition(
            RPL2,
            RSL2,
            updateSubsystemStates(RSL1)
        );
        transitionMap.addTransition(
            RPL3,
            RSL3,
            updateSubsystemStates(RSL1)
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(RPL1),
            RPL1,
            RSL2, RSL3, RSL4, NEUTRAL, INDEXING, SOURCE, INTAKE, EJECT_OUTTAKE, PROCESSOR_OUTTAKE 
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(RPL2),
            RPL2,
            RSL1, RSL3, RSL4, NEUTRAL, INDEXING, SOURCE, INTAKE, EJECT_OUTTAKE, PROCESSOR_OUTTAKE 
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(RPL3),
            RPL3,
            RSL1, RSL2, RSL4, NEUTRAL, INDEXING, SOURCE, INTAKE, EJECT_OUTTAKE, PROCESSOR_OUTTAKE 
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(RPL4),
            RPL4,
            RSL1, RSL3, RSL2, NEUTRAL, INDEXING, SOURCE, INTAKE, EJECT_OUTTAKE, PROCESSOR_OUTTAKE 
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(NEUTRAL),
            NEUTRAL,
            RPL1, RSL1, RSL3, RSL2, INDEXING, SOURCE, INTAKE, EJECT_OUTTAKE, PROCESSOR_PRIME, PROCESSOR_OUTTAKE, CLIMB_PRIME, CLIMB_LOCK, IDLE
        );
        transitionMap.addCommutativeTransition(
            SOURCE,
            INDEXING,
            updateSubsystemStates(INDEXING),
            updateSubsystemStates(NEUTRAL)
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(SOURCE),
            SOURCE,
            RSL1, RSL2, RSL3, RSL4, IDLE, INDEXING, INTAKE, EJECT_OUTTAKE, PROCESSOR_OUTTAKE
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(EJECT_OUTTAKE),
            EJECT_OUTTAKE,
            RSL1, RSL2, RSL3, RSL4, NEUTRAL, SOURCE, INTAKE
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(PROCESSOR_PRIME),
            PROCESSOR_PRIME,
            RSL1, RSL2, RSL3, RSL4, NEUTRAL, SOURCE, INTAKE
        );
        transitionMap.addTransition(
            PROCESSOR_PRIME,
            PROCESSOR_OUTTAKE,
            updateSubsystemStates(PROCESSOR_OUTTAKE)
        );
        transitionMap.addConvergingTransitions(
            updateSubsystemStates(CLIMB_PRIME),
            CLIMB_PRIME,
            RSL1, RSL2, RSL3, RSL4, NEUTRAL, SOURCE, INTAKE, EJECT_OUTTAKE, PROCESSOR_OUTTAKE, CLIMB_LOCK
        );
        transitionMap.addTransition(
            CLIMB_PRIME,
            CLIMB_LOCK,
            updateSubsystemStates(CLIMB_LOCK)
        );
        transitionMap.addTransition(
            CLIMB_LOCK,
            CLIMB_WINCH,
            updateSubsystemStates(CLIMB_WINCH)
        );


	}
}
