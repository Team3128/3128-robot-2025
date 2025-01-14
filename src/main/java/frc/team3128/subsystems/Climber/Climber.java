package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Climber.ClimberStates.*;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    private static Climber instance;

    protected WinchMechanism winch;

    private static TransitionMap<ClimberStates> transitionMap = new TransitionMap<ClimberStates>(ClimberStates.class);

    public Climber() {
        super(ClimberStates.class, transitionMap, IDLE);
        winch = new WinchMechanism();
        addSubsystem(winch);
        registerTransitions();
        System.out.println(transitionMap);
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addConvergingTransition(
            IDLE,
            sequence(
            runOnce(()-> setNeutralMode(COAST)),
            runOnce(()-> winch.stop())
        )
        );
	}


}
