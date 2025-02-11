package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.core.subsystems.NAR_PIDSubsystem.SetpointTest;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;

import static frc.team3128.subsystems.Climber.ClimberStates.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.team3128.Constants.TestingConstants.*;

import java.util.function.Function;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    private static Climber instance;

    public WinchMechanism winch;

    private static TransitionMap<ClimberStates> transitionMap = new TransitionMap<ClimberStates>(ClimberStates.class);

    // private Function<Neutral, Command> setNeutralMode = mode -> runOnce(() -> getMechanisms().forEach(subsystem -> subsystem.setNeutralMode(mode)));
    // private Function<ClimberStates, Command> transitioner = state -> {
    //     return sequence(
    //         setNeutralMode.apply(BRAKE),
    //         winch.pidTo(state.getAngle())
    //         runOnce(()->winch.lockServo.setPosition(state.getHasClaw() ? 1 : 0)),
    //         runOnce(()->winch.winchServo.setPosition(state.getHasWinch() ? 1 : 0))
    //     );
    // };

    public Climber() {
        super(ClimberStates.class, transitionMap, UNDEFINED);
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    public SystemsTest getClimberTest(ClimberStates state){
       return new SystemsTest(
            "Climb Test: " + state, 
            setStateCommand(state).withTimeout(CLIMBER_TEST_TIMEOUT), 
            ()-> atState()
        );
    }

    public SystemsTest getClimberTestNeutral(ClimberStates state){
       return new SystemsTest(
            "Climb Test: " + state, 
            sequence(setStateCommand(NEUTRAL), waitSeconds(2), setStateCommand(state).withTimeout(CLIMBER_TEST_TIMEOUT)), 
            ()-> atState()
        );
    }

    public boolean atState(){
        return WinchMechanism.controller.atSetpoint();
    }
    
    // private void addClimberTests() {
    //     Tester tester = Tester.getInstance();
    //     for(ClimberStates state : ClimberStates.values()){
    //         tester.addTest("Climber", getClimberTestNeutral(state));
    //     }
    // }

	@Override
	public void registerTransitions() {

        // //ALL STATES -> UNDEFINED
        // transitionMap.addConvergingTransition(
        //     UNDEFINED,
        //     sequence(
        //     runOnce(()-> setNeutralMode(COAST))//,
        //     // runOnce(()-> winch.stop())
        // )
        // );

        // //UNDEFINED, PRIME, LOCKED -> NEUTRAL
        // transitionMap.addConvergingTransitions(
        //     transitioner.apply(NEUTRAL), 
        //     NEUTRAL, 
        //     UNDEFINED, CLIMB_PRIME, CLIMB_LOCKED
        // );

        // //NEUTRAL -> PRIME
        // transitionMap.addTransition(NEUTRAL, CLIMB_PRIME, transitioner.apply(CLIMB_PRIME));

        // //PRIME -> LOCKED
        // transitionMap.addTransition(CLIMB_PRIME, CLIMB_LOCKED, transitioner.apply(CLIMB_LOCKED));

        // //LOCKED -> WINCH
        // transitionMap.addTransition(CLIMB_LOCKED, CLIMB_WINCH, transitioner.apply(CLIMB_WINCH));

        // //WINCH -> PRIME
        // transitionMap.addTransition(CLIMB_LOCKED, CLIMB_PRIME, transitioner.apply(CLIMB_PRIME));
	}
}
