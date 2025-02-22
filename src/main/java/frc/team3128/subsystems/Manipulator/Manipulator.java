package frc.team3128.subsystems.Manipulator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Intake.PivotMechanism;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.team3128.subsystems.Manipulator.ManipulatorStates.*;

import static frc.team3128.Constants.TestingConstants.*;

import java.util.function.Function;

public class Manipulator extends FSMSubsystemBase<ManipulatorStates> {
    private static Manipulator instance;

    public RollerMechanism roller;
    private static TransitionMap<ManipulatorStates> transitionMap = new TransitionMap<ManipulatorStates>(ManipulatorStates.class);
    private Function<ManipulatorStates, Command> defaultTransitioner = state -> {return runOnce(() -> roller.run(state.getPower()));};

    public Manipulator() {
        super(ManipulatorStates.class, transitionMap, NEUTRAL);
        roller = RollerMechanism.getInstance();
        addMechanisms(roller);
    }

    public static synchronized Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }
    public SystemsTest getManipulatorTest(ManipulatorStates state){
       return new SystemsTest(
            "Manipulator Test: " + state, 
            setStateCommand(state).withTimeout(MANIPULATOR_TEST_TIMEOUT), 
            ()-> atState()
        );
    }
    
    public SystemsTest getManipulatorTestNeutral(ManipulatorStates state){
        return new SystemsTest(
             "Manipulator Test: " + state, 
             sequence(setStateCommand(NEUTRAL), waitSeconds(MANIPULATOR_TEST_TIMEOUT), setStateCommand(state).withTimeout(MANIPULATOR_TEST_TIMEOUT)), 
             ()-> atState()
         );
     }

    public void addManipulatorTests() {
        Tester tester = Tester.getInstance();
        // for(ManipulatorStates state : ManipulatorStates.values()){
        //     if(state == NEUTRAL) tester.addTest("Manipulator", getManipulatorTest(NEUTRAL));
        //     else tester.addTest("Manipulator", getManipulatorTestNeutral(state));
        // }
        tester.addTest("Manipulator", getManipulatorTest(IN));
        tester.addTest("Manipulator", getManipulatorTest(NEUTRAL));
        tester.addTest("Manipulator", getManipulatorTest(OUT));
        tester.addTest("Manipulator", getManipulatorTest(NEUTRAL));
        tester.getTest("Manipulator").setTimeBetweenTests(1);
    }

    public boolean atState(){
        return Math.abs(RollerMechanism.leader.getAppliedOutput() - getState().getPower()) <= ROLLER_POWER_TOLERANCE;
    }
	@Override
	public void registerTransitions() {

        //ALL STATES -> UNDEFINED & UNDEFINED -> NEUTRAL
		transitionMap.addUndefinedState(UNDEFINED, NEUTRAL, stopCommand().andThen(()-> setNeutralMode(COAST)), defaultTransitioner.apply(NEUTRAL).beforeStarting(()-> setNeutralMode(BRAKE)));

        //NEUTRAL, IN, OUT are able to transition between each other
        transitionMap.addCommutativeTransition(functionalStates.asJava(), defaultTransitioner);
	}
}
