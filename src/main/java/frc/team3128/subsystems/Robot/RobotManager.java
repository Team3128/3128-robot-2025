package frc.team3128.subsystems.Robot;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Climber.WinchMechanism;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.Manipulator;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;

import java.util.function.BooleanSupplier;
import java.util.function.Function;

import static frc.team3128.Constants.TestingConstants.*;

public class RobotManager extends FSMSubsystemBase<RobotStates> {
    private static RobotManager instance;
    private static Elevator elevator;
    private static Manipulator manipulator;
    private static Intake intake;
    private static Climber climber;

    private static TransitionMap<RobotStates> transitionMap = new TransitionMap<>(RobotStates.class);
    private Function<RobotStates, Command> defaultTransitioner = state -> {return updateSubsystemStates(state);};

    private RobotManager() {
        super(RobotStates.class, transitionMap, UNDEFINED);

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

    public SystemsTest getRobotTest(RobotStates state){
        return new SystemsTest(
            "Robot Test: " + state, 
            updateSubsystemStates(state).withTimeout(ROBOT_TEST_TIMEOUT),
            ()-> atState()
        );
    }


    public SystemsTest getRobotTestNeutral(RobotStates state){
        return new SystemsTest(
            "Robot Test: " + state, 
            sequence(updateSubsystemStates(NEUTRAL), waitSeconds(ROBOT_TEST_TIMEOUT), updateSubsystemStates(state).withTimeout(ROBOT_TEST_TIMEOUT)),
            ()-> atState()
        );
    }

    public SystemsTest getRobotTestWait(RobotStates state){
        return new SystemsTest(
            "Robot Test: " + state, 
            sequence(waitSeconds(ROBOT_TEST_TIMEOUT), updateSubsystemStates(state).withTimeout(ROBOT_TEST_TIMEOUT)),
            ()-> atState()
        );
    }
    public void addRobotTests() {
        Tester tester = Tester.getInstance();
        // All Subsystem Individual Tests
        tester.addTest("Robot", tester.getTest("Intake"));
        tester.addTest("Robot", tester.getTest("Manipulator"));
        tester.addTest("Robot", tester.getTest("Elevator"));
        tester.addTest("Robot", tester.getTest("Climber"));
        // Coral Intake and L4-L1 then Score
        for(int i = 4; i > defaultElevatorStates.size(); i--){
            if (i == 4) { tester.addTest("Robot", getRobotTestNeutral(defaultElevatorStates.get(i))); }
            else { tester.addTest("Robot", getRobotTest(defaultElevatorStates.get(i))); }
        }
        tester.addTest("Robot", getRobotTest(RSL1));
        // Algae Intake and Outtake
        tester.addTest("Robot", getRobotTestNeutral(INTAKE));
        tester.addTest("Robot", getRobotTestWait(EJECT_OUTTAKE));
        // Climb
        tester.addTest("Robot", getRobotTestNeutral(CLIMB_PRIME));
        tester.addTest("Robot", getRobotTest(CLIMB));
        // Time
        tester.getTest("Robot").setTimeBetweenTests(1);        
    }


    public boolean atState(){
        return (elevator.atState() && climber.atState() && intake.atState() && manipulator.atState());
    }

    public Command updateSubsystemStates(RobotStates nextState) {
        return sequence(
            elevator.setStateCommand(nextState.getElevatorState()),
            manipulator.setStateCommand(nextState.getManipulatorState()),
            intake.setStateCommand(nextState.getIntakeState()),
            // climber.setStateCommand(nextState.getClimberState())
            runOnce(()-> Swerve.getInstance().throttle = nextState.getThrottle())
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

    public Command getTempToggleCommand(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition, double delay) {
        return either(
            setStateCommand(exclusiveState)
            .withTimeout(delay)
            .andThen(setStateCommand(NEUTRAL)), 
            updateSubsystemStates(defaultState), 
            condition
        );
    }

    public Command getTempToggleCommand(RobotStates defaultStates, RobotStates exclusiveState, BooleanSupplier condition) {
        return getTempToggleCommand(defaultStates, exclusiveState, condition, 1);
    }

    public Command getTempToggleCommand(RobotStates defaultStates, RobotStates exclusiveState) {
        return getTempToggleCommand(defaultStates, exclusiveState,  ()-> stateEquals(defaultStates), 1);
    }

    public Command getToggleCommand(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        return either(
            updateSubsystemStates(exclusiveState), 
            updateSubsystemStates(defaultState), 
            condition
        );
    }

    public Command getToggleCommand(RobotStates defaultState, RobotStates exclusiveState) {
        return getToggleCommand(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command getToggleCommand(RobotStates state) {
        return getToggleCommand(state, NEUTRAL);
    }

    @Override
    public void registerTransitions() {
        
        // From all transitions to Undefined and Undefined to Neutral
        transitionMap.addUndefinedState(UNDEFINED, NEUTRAL, defaultTransitioner);

        // Between all default states
        transitionMap.addCommutativeTransition(defaultStates.asJava(), defaultTransitioner);

        // For each coupled states pair
        transitionMap.addMappedTransition(coupledStates.asJava(), defaultTransitioner);

        // From exclusive state to Neutral
        transitionMap.addConvergingTransition(exclusiveStates.asJava(), NEUTRAL, defaultTransitioner);

    }
}
