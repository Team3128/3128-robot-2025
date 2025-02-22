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
            roller.stopCommand(),
            winch.pidTo(state.getAngle()),
            waitUntil(()-> winch.atSetpoint()),
            roller.runCommand(state.getRollerPower())
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

    public boolean atTestState(){
        return WinchMechanism.controller.atSetpoint() && Math.abs(RollerMechanism.leader.getAppliedOutput() - getState().getRollerPower()) <= ROLLER_POWER_TOLERANCE;
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
