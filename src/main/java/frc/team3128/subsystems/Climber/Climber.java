package frc.team3128.subsystems.Climber;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Climber.ClimberStates.*;

public class Climber extends FSMSubsystemBase<ClimberStates> {
    private static Climber instance;

    private Winch winch;

    private static TransitionMap<ClimberStates> transitionMap;

    private Climber() {
        super(ClimberStates.class, transitionMap, IDLE);
        winch = new Winch();
        addSubsystem(winch);
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
		transitionMap.addConvergingTransition(IDLE, sequence(
                winch.stop(),
                runOnce(()-> winch.setNeutralMode(COAST))
        ));

        for(ClimberStates state : ClimberStates.values()) {
            if(state == IDLE) continue;

            transitionMap.addConvergingFunction(
                sequence(
                    runOnce(()-> winch.setNeutralMode(BRAKE)),
                    winch.pidTo(state.getSetpoint())
                ),
                state,
                NEUTRAL, SOURCE, L1, L2, L3, L4
            );    
        }

	}
}
