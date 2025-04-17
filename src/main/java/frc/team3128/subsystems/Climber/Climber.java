package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Climber.ClimberStates.*;

import java.util.List;
import java.util.function.Function;

import au.grapplerobotics.LaserCan;


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
            runOnce(() -> WinchMechanism.controller.getConfig().kS = () -> 12 * state.getWinchPower()),
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

        // initShuffleboard();
        registerTransitions();
        // NAR_Shuffleboard.addData("LaserCAN", "Measurment", ()-> roller.lc.getMeasurement().distance_mm, 0, 0);
        // NAR_Shuffleboard.addData("LaserCAN", "Sees Something", ()-> roller.lc.getMeasurement().distance_mm < 100, 1, 0);
        // NAR_Shuffleboard.addData("LaserCAN", "IsTriggered", ()-> roller.lc.getMeasurement().distance_mm < 100, 2, 0);

    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(NEUTRAL, PRE_CLIMB_PRIME, CLIMB_PRIME, CLIMB), defaultTransitioner);
	}
}
