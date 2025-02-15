package frc.team3128.subsystems.Elevator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Climber.WinchMechanism;
import frc.team3128.subsystems.Intake.PivotMechanism;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.team3128.subsystems.Elevator.ElevatorStates.*;
import static frc.team3128.Constants.TestingConstants.*;


import java.util.function.Function;

public class Elevator extends FSMSubsystemBase<ElevatorStates> {
    private static Elevator instance;

    protected ElevatorMechanism elevator;
    private static TransitionMap<ElevatorStates> transitionMap = new TransitionMap<ElevatorStates>(ElevatorStates.class);
    private Function<ElevatorStates, Command> defaultTransitioner = state -> {return elevator.pidTo(state.getSetpoint());};

    public Elevator() {
        super(ElevatorStates.class, transitionMap, NEUTRAL);
        elevator = ElevatorMechanism.getInstance();
        addMechanisms(elevator);
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }
    
    public SystemsTest getElevatorTest(ElevatorStates state){
       return new SystemsTest(
            "Elevator Test: " + state, 
            setStateCommand(state).withTimeout(ELEVATOR_TEST_TIMEOUT), 
            ()-> atState()
        );
    }
    public SystemsTest getElevatorTestNeutral(ElevatorStates state){
        return new SystemsTest(
             "Elevator Test: " + state, 
             sequence(setStateCommand(NEUTRAL), waitSeconds(ELEVATOR_TEST_TIMEOUT), setStateCommand(state).withTimeout(ELEVATOR_TEST_TIMEOUT)), 
             ()-> atState()
         );
     }


    public boolean atState(){
        return ElevatorMechanism.controller.atSetpoint();
    }

    public void addElevatorTests() {
        Tester tester = Tester.getInstance();
        for(ElevatorStates state : ElevatorStates.values()){
            if(state == NEUTRAL) tester.addTest("Elevator", getElevatorTest(NEUTRAL));
            else tester.addTest("Elevator", getElevatorTestNeutral(state));
        }
        tester.getTest("Elevator").setTimeBetweenTests(1);
    }

	@Override
	public void registerTransitions() {

        //ALL STATES -> UNDEFINED & UNDEFINED -> NEUTRAL
		transitionMap.addUndefinedState(UNDEFINED, NEUTRAL, stopCommand().andThen(()-> setNeutralMode(COAST)), defaultTransitioner.apply(NEUTRAL).beforeStarting(()-> setNeutralMode(BRAKE)));

        transitionMap.addCommutativeTransition(functionalStates.asJava(), defaultTransitioner);
	}
}
