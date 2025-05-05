package frc.team3128.subsystems.Leds;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;

public class Leds extends FSMSubsystemBase<LedsStates> {
    
    private static Leds instance;
    private static TransitionMap<LedsStates> transitionMap = new TransitionMap<LedsStates>(LedsStates.class);


    public static synchronized Leds getInstance() {
        if (instance == null)
            instance = new Leds();
        return instance;
    }

    public Leds() {
        super(LedsStates.class, transitionMap, LedsStates.DISABLED);
    }

    @Override
    public void registerTransitions() {
        
    }
}
