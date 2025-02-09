package frc.team3128.subsystems.Led;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3128.commands.CmdAlignReef;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;

import static frc.team3128.Constants.LedConstants.*;
import static frc.team3128.subsystems.Led.LedStates.*;

import java.util.List;

public class Led extends FSMSubsystemBase<LedStates> {

    public static Led instance;

    private final CANdle m_candle = new CANdle(CANDLE_ID);

    private static TransitionMap<LedStates> transitionMap = new TransitionMap<LedStates>(LedStates.class);


    public static synchronized Led getInstance() {
        if (instance == null) {
            instance = new Led();
        }

        return instance;
    }

    private Led() {
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
        transitionMap.addConvergingTransition(PROXIMITY, new CmdAlignReef());
        transitionMap.addCommutativeTransition(List.of(DISABLED, ENABLED, AUTO), state -> runOnce(() -> setLedColor(state)));
        transitionMap.addDivergingTransition(PROXIMITY, state -> runOnce(() -> setLedColor(state)).beforeStarting(() -> CommandScheduler.getInstance().requiring(Led.getInstance()).cancel()));
    }

    //Set Elevator Leds
    public void setLedColor(LedStates ledState) {
        setLedColor(ledState, 1);
    }

    public void setLedColor(LedStates ledState, double percent) {
        m_candle.setLEDs(ledState.getColor().r, ledState.getColor().g, ledState.getColor().b, 0, 4, MathUtil.clamp((int) (percent * 27), 0, 27));
    }

    public void setLedColor(Colors color, double percent) {
        m_candle.setLEDs(color.r, color.g, color.b, 0, 4, MathUtil.clamp((int) (percent * 27), 0, 27));
    }

    public void resetAnimationSlot(int slots) {
        m_candle.setLEDs(0,0,0, WHITE_VALUE, STARTING_ID, PIVOT_COUNT);
        for (int i = 0; i < slots; i++) {
         m_candle.animate(null, i);
        }
    }
}