package frc.team3128.subsystems.Led;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;

import static frc.team3128.Constants.LedConstants.*;
import static frc.team3128.subsystems.Led.LedStates.*;

public class Led extends FSMSubsystemBase<LedStates> {
    private static Led instance;
    private final CANdle m_candle = new CANdle(CANDLE_ID);

    private static TransitionMap<LedStates> transitionMap = new TransitionMap<LedStates>(LedStates.class);


    public static synchronized Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }

        return instance;
    }

    public Led() {
        super(LedStates.class, transitionMap);
        configCandle();   
    }

    private void configCandle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.statusLedOffWhenActive = true;
        config.brightnessScalar = 1;
        m_candle.configAllSettings(config);
    }

    @Override
    public void registerTransitions() {
        transitionMap.addCommutativeTransition(functionalStates.asJava(), state -> {return runOnce(()-> setLedColor(state));});
    }

    //Set Elevator Leds
    public void setLedColor(LedStates ledState) {
        if (ledState != LedStates.PROXIMITY) {
            resetAnimationSlot(2);

            if (ledState.getColor().animation) {
                if (ledState.getColor() == Colors.FLAME) {
                    m_candle.animate(new FireAnimation(BRIGHTNESS, r_SPEED, NUM_LED, SPARKING, COOLING), 1);
                }
            } else {
                m_candle.animate(new ColorFlowAnimation(ledState.getColor().r, ledState.getColor().g, ledState.getColor().b, WHITE_VALUE, r_SPEED, NUM_LED, ColorFlowAnimation.Direction.Forward, 5), 0);
            }
        } else {
            //help
            //level raise with closeness yes
        }
    }

    public void resetAnimationSlot(int slots) {
        m_candle.setLEDs(0,0,0, WHITE_VALUE, STARTING_ID, PIVOT_COUNT);
        for (int i = 0; i < slots; i++) {
         m_candle.animate(null, i);
        }
    }
}