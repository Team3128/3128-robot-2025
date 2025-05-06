package frc.team3128.subsystems.Leds;

import java.util.List;
import java.util.function.Function;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;

public class Leds extends FSMSubsystemBase<LedsStates> {
    
    private static Leds instance;
    private static TransitionMap<LedsStates> transitionMap = new TransitionMap<LedsStates>(LedsStates.class);

 private Function<LedsStates, Command> defaultTransitioner = state -> {return Commands.runOnce(()->LedsMechanism.getInstance().setColor(state.getColor()));};

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
         //NEUTRAL, IN, OUT are able to transition between each other
        transitionMap.addCommutativeTransition(List.of(LedsStates.values()), defaultTransitioner);
    }
}
