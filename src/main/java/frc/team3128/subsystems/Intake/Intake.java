package frc.team3128.subsystems.Intake;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;

import static frc.team3128.subsystems.Intake.IntakeStates.*;
import java.util.function.Function;

import static frc.team3128.Constants.TestingConstants.*;

public class Intake extends FSMSubsystemBase<IntakeStates> {
    
    private static Intake instance;

    protected PivotMechanism pivot;
    protected RollerMechanism roller;
    private static TransitionMap<IntakeStates> transitionMap = new TransitionMap<IntakeStates>(IntakeStates.class);
    private Function<IntakeStates, Command> defaultTransitioner = state -> {
        return sequence(
            PivotMechanism.getInstance().pidTo(state.getAngle()),
            RollerMechanism.getInstance().runCommand(state.getPower())
        );
    };

    public Intake() {
        super(IntakeStates.class, transitionMap, NEUTRAL);

        pivot = PivotMechanism.getInstance();
        roller = RollerMechanism.getInstance();

        addMechanisms(roller);
    }

    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

    public SystemsTest getIntakeTest(IntakeStates state){
       return new SystemsTest(
            "Intake Test: " + state, 
            setStateCommand(state).withTimeout(INTAKE_TEST_TIMEOUT), 
            ()-> atState()
        );
    }
    
    public SystemsTest getIntakeTestNeutral(IntakeStates state){
        return new SystemsTest(
             "Intake Test: " + state, 
             sequence(setStateCommand(NEUTRAL), waitSeconds(INTAKE_TEST_TIMEOUT), setStateCommand(state).withTimeout(INTAKE_TEST_TIMEOUT)), 
             ()-> atState()
         );
     }

     public SystemsTest getIntakeTestWait(){
        return new SystemsTest(
             "Intake Test: ", 
             sequence(waitSeconds(INTAKE_TEST_TIMEOUT), setStateCommand(INTAKE).withTimeout(INTAKE_TEST_TIMEOUT)), 
             ()-> atState()
         );
     }

    public boolean atState(){
        return PivotMechanism.controller.atSetpoint() && Math.abs(RollerMechanism.leader.getAppliedOutput() - getState().getPower()) <= ROLLER_POWER_TOLERANCE;
    }

    public void addIntakeTests() {
        Tester tester = Tester.getInstance();
        // for(IntakeStates state : IntakeStates.values()){
        //     if(state == NEUTRAL || state == CLIMB) tester.addTest("Intake", getIntakeTest(state));
        //     else tester.addTest("Intake", getIntakeTestNeutral(state));
        // }
        tester.addTest("Intake", getIntakeTest(INTAKE));
        tester.addTest("Intake", getIntakeTest(NEUTRAL));
        tester.addTest("Intake", getIntakeTest(EJECT_OUTTAKE));
        tester.addTest("Intake", getIntakeTest(NEUTRAL));
        tester.getTest("Intake").setTimeBetweenTests(1);
    }

	@Override
	public void registerTransitions() {
        //ALL STATES -> UNDEFINED
		transitionMap.addUndefinedState(UNDEFINED, NEUTRAL, stopCommand().andThen(()-> setNeutralMode(COAST)), defaultTransitioner.apply(NEUTRAL).beforeStarting(()-> setNeutralMode(BRAKE)));

        //DEFAULT STATES -> DEFAULT STATES
        transitionMap.addCommutativeTransition(defaultStates.asJava(), defaultTransitioner);

        //EXCLUSIVE STATES -> NEUTRAL
        transitionMap.addConvergingTransition(exclusiveStates.asJava(), NEUTRAL, defaultTransitioner);

        //PRIME STATES -> OUTPUT STATES
        transitionMap.addMappedTransition(coupledStates.asJava(), defaultTransitioner);


	}
}
