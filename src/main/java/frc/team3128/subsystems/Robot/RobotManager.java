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
import edu.wpi.first.wpilibj.DriverStation;

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
        super(RobotStates.class, transitionMap, NEUTRAL);

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
            climber.setStateCommand(nextState.getClimberState()),
            waitUntil(()-> climber.winch.atSetpoint()),
            runOnce(()-> Swerve.getInstance().throttle = nextState.getThrottle()).onlyIf(()-> DriverStation.isTeleop())
        );
    }

    public Command getTempToggleCommand(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition, double delay) {
        return either(
            sequence(
                setStateCommand(exclusiveState),
                waitSeconds(0.5),
                setStateCommand(NEUTRAL)
            ),
            setStateCommand(defaultState), 
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
            setStateCommand(exclusiveState), 
            setStateCommand(defaultState), 
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

    public SystemsTest getRobotTest(String name, RobotStates state, double timeout, BooleanSupplier atTestState){
        return new SystemsTest(
            "Test - " + name, 
            updateSubsystemStates(state).withTimeout(timeout),
            atTestState
        );
    }

    public void addCoralTest(){
        Tester tester = Tester.getInstance();
        
        tester.addTest("Coral", getRobotTest("Coral", NEUTRAL, CORAL_TEST_TIMEOUT, ()-> elevator.atTestState() && manipulator.atTestState()));
        tester.addTest("Coral", getRobotTest("Coral", RPL2, CORAL_TEST_TIMEOUT, ()-> elevator.atTestState() && manipulator.atTestState()));
        tester.addTest("Coral", getRobotTest("Coral", RSL2, CORAL_TEST_TIMEOUT, ()-> elevator.atTestState() && manipulator.atTestState()));
        tester.addTest("Coral", getRobotTest("Coral", NEUTRAL, CORAL_TEST_TIMEOUT, ()-> elevator.atTestState() && manipulator.atTestState()));
        tester.getTest("Coral").setTimeBetweenTests(2);   
    }

    public void addClimberTest(){
        Tester tester = Tester.getInstance();

        tester.addTest("Climber", getRobotTest("Climber", NEUTRAL, CLIMBER_TEST_TIMEOUT, ()-> climber.atTestState()));
        tester.addTest("Climber", getRobotTest("Climber", CLIMB_PRIME, CLIMBER_TEST_TIMEOUT, ()-> climber.atTestState()));
        tester.addTest("Climber", getRobotTest("Climber", CLIMB, CLIMBER_TEST_TIMEOUT, ()-> climber.atTestState()));
        tester.addTest("Climber", getRobotTest("Climber", NEUTRAL, CLIMBER_TEST_TIMEOUT, ()-> climber.atTestState()));
        tester.getTest("Climber").setTimeBetweenTests(2);
    }

    public void addIntakeTest(){
        Tester tester = Tester.getInstance();

        tester.addTest("Intake", getRobotTest("Intake", NEUTRAL, INTAKE_TEST_TIMEOUT, ()-> intake.atTestState()));
        tester.addTest("Intake", getRobotTest("Intake", INTAKE, INTAKE_TEST_TIMEOUT, ()-> intake.atTestState()));
        tester.addTest("Intake", getRobotTest("Intake", NEUTRAL, INTAKE_TEST_TIMEOUT, ()-> intake.atTestState()));
        tester.addTest("Intake", getRobotTest("Intake", EJECT_OUTTAKE, INTAKE_TEST_TIMEOUT, ()-> intake.atTestState()));
        tester.addTest("Intake", getRobotTest("Intake", NEUTRAL, INTAKE_TEST_TIMEOUT, ()-> intake.atTestState()));
        tester.getTest("Intake").setTimeBetweenTests(2);
    }

    public void addRobotTests() {
        Tester tester = Tester.getInstance();

        tester.addTest("Robot", tester.getTest("Coral"));
        tester.addTest("Robot", tester.getTest("Climber"));
        tester.addTest("Robot", tester.getTest("Intake"));
        tester.getTest("Robot").setTimeBetweenTests(2);        
    }
}
