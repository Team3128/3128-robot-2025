package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.core.subsystems.NAR_PIDSubsystem.SetpointTest;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.tester.Tester;
import common.utility.tester.Tester.SystemsTest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;

import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Climber.ClimberStates.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.team3128.Constants.TestingConstants.*;

import java.util.List;
import java.util.function.Function;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    private static Climber instance;

    public WinchMechanism winch;
    public RollerMechanism roller;

    private static TransitionMap<ClimberStates> transitionMap = new TransitionMap<ClimberStates>(ClimberStates.class);

    private Function<Neutral, Command> setNeutralMode = mode -> runOnce(() -> getMechanisms().forEach(subsystem -> subsystem.setNeutralMode(mode)));
    private Function<ClimberStates, Command> defaultTransitioner = state -> {
        return sequence(
            none(),
            roller.runCommand(state.getRollerPower()),
            winch.pidTo(state.getAngle())
        );
    };

    public Climber() {
        super(ClimberStates.class, transitionMap, NEUTRAL);

        winch = WinchMechanism.getInstance();
        roller = RollerMechanism.getInstance();

        addMechanisms(winch, roller);
        // addMechanisms(winch);

        initShuffleboard();
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
            sequence(setStateCommand(NEUTRAL), waitSeconds(CLIMBER_TEST_TIMEOUT), setStateCommand(state).withTimeout(CLIMBER_TEST_TIMEOUT)), 
            ()-> atState()
        );
    }

    public boolean atState(){
        return WinchMechanism.controller.atSetpoint();
    }
    
    public void addClimberTests() {
        Tester tester = Tester.getInstance();
        // for(ClimberStates state : ClimberStates.values()){
        //     if(state == NEUTRAL || state == CLIMB) tester.addTest("Climber", getClimberTest(state));
        //     else tester.addTest("Climber", getClimberTestNeutral(state));
        // }
        tester.addTest("Climber", getClimberTest(CLIMB_PRIME));
        tester.addTest("Climber", getClimberTest(NEUTRAL));
        tester.addTest("Climber", getClimberTest(CLIMB));
        tester.addTest("Climber", getClimberTest(NEUTRAL));
        tester.getTest("Climber").setTimeBetweenTests(1);
    }

	@Override
	public void registerTransitions() {
        transitionMap.addUndefinedState(
            UNDEFINED, 
            NEUTRAL, 
            stopCommand().andThen(() -> setNeutralMode(Neutral.COAST)),
            defaultTransitioner.apply(NEUTRAL).beforeStarting(() -> setNeutralMode(Neutral.BRAKE))
        );

        transitionMap.addCommutativeTransition(List.of(NEUTRAL, CLIMB_PRIME, CLIMB), defaultTransitioner);
	}
}
