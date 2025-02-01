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
import io.vavr.collection.List;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.RobotContainer.printStatus;
import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class RobotManager extends FSMSubsystemBase<RobotStates> {
    private static RobotManager instance;
    private static Elevator elevator;
    private static Manipulator manipulator;
    private static Intake intake;
    private static Climber climber;

    private static TransitionMap<RobotStates> transitionMap = new TransitionMap<>(RobotStates.class);
    private Function<RobotStates, Command> defaultTransitioner = state -> {return updateSubsystemStates(state);};

    private static boolean hasObjectPresent;

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
            elevator.setStateCommand(nextState.getElevatorState()),
            manipulator.setStateCommand(nextState.getManipulatorState()),
            intake.setStateCommand(nextState.getIntakeState()),
            climber.setStateCommand(nextState.getClimberState())
        );
    }

    public Command getCoralState(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        return either(
            setStateCommand(exclusiveState)
            // .until(()-> manipulator.hasObjectPresent() != hasObjectPresent)
            .andThen(waitSeconds(1))
            .andThen(setStateCommand(NEUTRAL)),
            setStateCommand(defaultState),
            condition
        );
    }

    public Command getCoralState(RobotStates defaultState, RobotStates exclusiveState) {
        return getCoralState(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command getAlgaeState(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        return either(
            setStateCommand(exclusiveState)
            .withTimeout(1)
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
        
        // From all transitions to Idle
        transitionMap.addConvergingTransition(IDLE, defaultTransitioner);

        // From Idle to Neutral
        transitionMap.addTransition(IDLE, NEUTRAL, defaultTransitioner);

        // Between all default states
        transitionMap.addCommutativeTransition(defaultStates.asJava(), defaultTransitioner);

        // From reef primes to reef scores
        transitionMap.addCorrespondenceTransitions(defaultElevatorStates.asJava(), exclusiveElevatorStates.asJava(), defaultTransitioner);

        // From climb prime to climb lock and climb score
        transitionMap.addCorrespondenceTransitions(List.fill(exclusiveClimbStates.size(), CLIMB_PRIME).asJava(), exclusiveClimbStates.asJava(), defaultTransitioner);

        // From processor prime to processor outtake
        transitionMap.addTransition(PROCESSOR_PRIME, PROCESSOR_OUTTAKE, defaultTransitioner);

        // From exclusive state to Neutral
        transitionMap.addConvergingTransition(exclusiveStates.asJava(), NEUTRAL, defaultTransitioner);

    }
}
