package frc.team3128.subsystems.Intake;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.Log;

import static frc.team3128.subsystems.Intake.IntakeStates.*;
import java.util.function.Function;

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
        Log.profile("Intake State Machine Register Transitions", ()-> registerTransitions());
    }

    public static synchronized Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }

	@Override
	public void registerTransitions() {

        //DEFAULT STATES -> DEFAULT STATES
        transitionMap.addCommutativeTransition(defaultStates.asJava(), defaultTransitioner);

        //EXCLUSIVE STATES -> NEUTRAL
        transitionMap.addConvergingTransition(exclusiveStates.asJava(), NEUTRAL, defaultTransitioner);

        //PRIME STATES -> OUTPUT STATES
        transitionMap.addMappedTransition(coupledStates.asJava(), defaultTransitioner);


	}
}
