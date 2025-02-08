package frc.team3128.subsystems.Manipulator;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import frc.team3128.doglog.DogLog;

import static common.hardware.motorcontroller.NAR_Motor.Neutral.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Manipulator.ManipulatorStates.*;

public class Manipulator extends FSMSubsystemBase<ManipulatorStates> {
    private static Manipulator instance;

    public RollerMechanism roller;
    private static TransitionMap<ManipulatorStates> transitionMap = new TransitionMap<ManipulatorStates>(ManipulatorStates.class);

    public Manipulator() {
        super(ManipulatorStates.class, transitionMap, IDLE);
        roller = new RollerMechanism();
        addSubsystem(roller);
        registerTransitions();

        System.out.println(transitionMap);
    }

    public static synchronized Manipulator getInstance() {
        if (instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

	@Override
	public void registerTransitions() {

        //ALL STATES -> IDLE
		transitionMap.addConvergingTransition(IDLE, sequence(
                roller.stopCommand(),
                runOnce(()-> setNeutralMode(COAST))
        ));

        //NEUTRAL, FORWARD, REVERSE are able to transition between each other
        transitionMap.applyCommutativeFunction(
            state -> {
                return sequence(
                    runOnce(()-> setNeutralMode(BRAKE)),
                    run(state.getPower())
                );
            }, 
            functionalStates
        );

        //IDLE -> NEUTRAL
        transitionMap.addTransition(
            IDLE, 
            NEUTRAL, 
            sequence(
                    runOnce(()-> setNeutralMode(BRAKE)),
                    run(NEUTRAL.getPower())
            )
        );
	}

    public boolean hasObjectPresent() {
        return roller.hasObjectPresent();
    }

    public void dogLogPeriodic(){
        DogLog.log("Manipulator State", getState());
        DogLog.log(getName() + "hasObjectPresent", hasObjectPresent());
        roller.dogLogPeriodic();
    }
}
