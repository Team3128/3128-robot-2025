package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Climber.ClimberStates.*;

import java.util.List;
import java.util.function.Function;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    private static Climber instance;

    public WinchMechanism winch;
    public RollerMechanism roller;

    private static TransitionMap<ClimberStates> transitionMap = new TransitionMap<ClimberStates>(ClimberStates.class);

    private Function<Neutral, Command> setNeutralMode = mode -> runOnce(() -> getMechanisms().forEach(subsystem -> subsystem.setNeutralMode(mode)));
    private Function<ClimberStates, Command> defaultTransitioner = state -> {
        return Commands.sequence(
            setNeutralMode.apply(Neutral.BRAKE),
            RollerMechanism.getInstance().runCommand(state.getRollerPower()),
            sequence(
                WinchMechanism.getInstance().runCommand(state.getWinchPower()),
                waitUntil(()->WinchMechanism.getInstance().atSetpoint(state.getAngle())),
                WinchMechanism.getInstance().runCommand(0)
            )

        );
    };

    public Climber() {
        super(ClimberStates.class, transitionMap, UNDEFINED);

        winch = WinchMechanism.getInstance();
        roller = RollerMechanism.getInstance();

        addMechanisms(winch,roller);
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
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
