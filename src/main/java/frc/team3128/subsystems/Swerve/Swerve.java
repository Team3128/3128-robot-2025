package frc.team3128.subsystems.Swerve;

import java.util.function.Function;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import static frc.team3128.subsystems.Swerve.SwerveStates.*;
import edu.wpi.first.wpilibj2.command.Command;

public class Swerve extends FSMSubsystemBase<SwerveStates> {
    private static Swerve instance;

    protected SwerveMechanism swerve;
    private static TransitionMap<SwerveStates> transitionMap = new TransitionMap<SwerveStates>(SwerveStates.class);
    private Function<SwerveStates, Command> defaultTransitioner = state -> {return swerve.pidTo(state.getThrottle());};

    public Swerve() {
        super(SwerveStates.class, transitionMap, NEUTRAL);
        swerve = SwerveMechanism.getInstance();
        addMechanisms(swerve);
        
    }

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }

        return instance;
    }

	@Override
	public void registerTransitions() {
        transitionMap.addCommutativeTransition(functionalStates.asJava(), defaultTransitioner);
	}
}
