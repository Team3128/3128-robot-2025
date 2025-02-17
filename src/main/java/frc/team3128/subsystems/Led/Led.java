package frc.team3128.subsystems.Led;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.led.RainbowAnimation;
import frc.team3128.commands.CmdAlignReef;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;

import static frc.team3128.Constants.LedConstants.*;
import static frc.team3128.Constants.SwerveConstants.DRIVETRAIN_CANBUS_NAME;
import static frc.team3128.subsystems.Led.LedStates.*;

import java.util.List;

public class Led extends FSMSubsystemBase<LedStates> {

    public static Led instance;

    private final CANdle candle = new CANdle(CANDLE_ID, DRIVETRAIN_CANBUS_NAME);

    private static TransitionMap<LedStates> transitionMap = new TransitionMap<LedStates>(LedStates.class);


    public static synchronized Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }

        return instance;
    }

    private Led() {
        super(LedStates.class, transitionMap, NEUTRAL);
        configCandle();   
    }

    private void configCandle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.statusLedOffWhenActive = true;
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }

    @Override
    public void registerTransitions() {
        transitionMap.addConvergingTransition(REEF_PRIME, new CmdAlignReef());

        transitionMap.addCommutativeTransition(List.of(UNDEFINED, NEUTRAL, REEF_SCORE), state -> runOnce(() -> setLedColor(state)));
        
        transitionMap.addDivergingTransition(REEF_PRIME, state -> runOnce(() -> setLedColor(state)).beforeStarting(() -> CommandScheduler.getInstance().requiring(Led.getInstance()).cancel()));
    }

    //Set Elevator Leds
    public void setLedColor(LedStates ledState) {
        resetAnimationSlot();

        switch (ledState) {
            case UNDEFINED:
                candle.animate(new RainbowAnimation(BRIGHTNESS, r_SPEED, NUM_LED, false, STARTING_ID), 0);
                break;
            case NEUTRAL:
                candle.animate(new FireAnimation(BRIGHTNESS, r_SPEED, NUM_LED, SPARKING, COOLING, false, STARTING_ID), 0);
                break;
            case REEF_SCORE:
                candle.setLEDs(ledState.r, ledState.g, ledState.b, WHITE_VALUE, STARTING_ID, NUM_LED);
            default:
                break;
        }
    }

    public void setLedColor(LedStates ledState, double percent) {
        candle.setLEDs(ledState.r, ledState.g, ledState.b, WHITE_VALUE, STARTING_ID, MathUtil.clamp((int) (percent * NUM_LED), 0, NUM_LED));
    }

    public void resetAnimationSlot() {
        candle.setLEDs(0,0,0, WHITE_VALUE, STARTING_ID, NUM_LED);
        candle.animate(null, 0);
    }
}