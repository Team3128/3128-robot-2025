package frc.team3128.subsystems.Led;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3128.subsystems.Swerve;
import static frc.team3128.subsystems.Led.LedStates.*;

import java.util.List;


public class Led extends FSMSubsystemBase<LedStates>{
    
    private static Led instance;

    public static synchronized Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }

        return instance;
    }

    public LedMechanism led = LedMechanism.getInstance();
    private static TransitionMap<LedStates> transitionMap = new TransitionMap<LedStates>(LedStates.class);

    public Led() {
        super(LedStates.class, transitionMap, DISABLED);
    }

    @Override
    public void registerTransitions() {
        transitionMap.addCommutativeTransition(List.of(LedStates.values()), (state)-> defaultTransitioner(state));

        transitionMap.addConvergingTransition(List.of(LedStates.values()), DEFAULT, Commands.run(
            ()-> led.setColor(Swerve.getInstance().getClosestReef().getLedStates().getColor())
        ));
    }

    private Command defaultTransitioner(LedStates nextState) {
        return Commands.runOnce(()-> {
            if(nextState.getAnimation() != null) {
                led.setAnimation(nextState.getAnimation());
            }
            else if(nextState.getColor() != null) {
                led.setColor(nextState.getColor());
            }
        });
    }

}
