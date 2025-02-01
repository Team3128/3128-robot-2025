package frc.team3128.subsystems.Intake;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import io.vavr.collection.List;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.Transition;
import common.core.fsm.TransitionMap;
import common.hardware.motorcontroller.NAR_Motor.Neutral;
import common.utility.Log;

import static frc.team3128.subsystems.Intake.IntakeStates.*;
import static frc.team3128.RobotContainer.printStatus;
import java.util.function.Function;

public class Intake extends FSMSubsystemBase<IntakeStates> {
    
    private static Intake instance;

    // protected PivotMechanism pivot;
    // protected RollerMechanism roller;
    private static TransitionMap<IntakeStates> transitionMap = new TransitionMap<IntakeStates>(IntakeStates.class);
    private Function<IntakeStates, Command> defaultTransitioner = state -> {
        return sequence(
            // pivot.pidTo(state.getAngle()),
            // roller.run(state.getPower())
        );
    };

    public Intake() {
        super(IntakeStates.class, transitionMap, NEUTRAL);
        // pivot = new PivotMechanism();
        // roller = new RollerMechanism();
        // addSubsystem(pivot, roller);
        // registerTransitions();
    }

    public static synchronized Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
        //ALL STATES -> IDLE
		transitionMap.addConvergingTransition(IDLE, defaultTransitioner.apply(IDLE).andThen(stopCommand()).andThen(runOnce(()-> setNeutralMode(COAST))));

        transitionMap.addCommutativeTransition(defaultIntakeStates.appendAll(defaultClimbStates).asJava(), defaultTransitioner);

        transitionMap.addConvergingTransition(exclusiveIntakeStates.appendAll(exclusiveClimbStates).asJava(), NEUTRAL, defaultTransitioner);

        transitionMap.addTransition(PROCESSOR_PRIME, PROCESSOR_OUTTAKE, defaultTransitioner);

        transitionMap.addCorrespondenceTransitions(List.fill(exclusiveClimbStates.size(), CLIMB_PRIME).asJava(), exclusiveClimbStates.asJava(), defaultTransitioner);
	}
}
