// package frc.team3128.subsystems.Leds;

// import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdle.LEDStripType;

// import common.core.subsystems.NAR_PIDSubsystem;
// import common.core.subsystems.NAR_Subsystem;
// import common.hardware.motorcontroller.NAR_Motor.Neutral;
// import common.utility.Log;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix.led.CANdleConfiguration;

// import static frc.team3128.Constants.LedsConstants.*;
// import static frc.team3128.Constants.SwerveConstants.*;

// public class LedsMechanism implements NAR_Subsystem {
    
//     private static LedsMechanism instance;

//     private final CANdle candle = new CANdle(CANDLE_ID, "drivetrain");

//     public static synchronized LedsMechanism getInstance() {
//         if (instance == null)
//             instance = new LedsMechanism();
//         return instance;
//     }

//     public LedsMechanism() {
//         configCandle();
//     }

//     private void configCandle() {
//         CANdleConfiguration config = new CANdleConfiguration();
//         config.stripType = LEDStripType.RGB;
//         config.disableWhenLOS = true; // turn off when signal lost
//         config.brightnessScalar = 1;
//         candle.configAllSettings(config);
//     }

//     public void reset() {
//         candle.setLEDs(0,0,0, 0, 0, MAX_HEIGHT);
//         candle.animate(null, 0);
//     }

//     public void setColor(Color color) {
//         setColor(color, MAX_HEIGHT);
//     }

//     public void setColor(Color color, int height) {
//         reset();
//         candle.setLEDs((int) (color.red * 256), (int) (color.green * 256), (int) (color.blue * 256), 0, 0, height);
//     }

//     public void setAnimation(Animation animation) {
//         reset();
//         candle.animate(animation, 0);
//     }

//     @Override
//     public Command resetCommand() {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'resetCommand'");
//     }

//     @Override
//     public void run(double power) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'run'");
//     }

//     @Override
//     public Command runCommand(double power) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'runCommand'");
//     }

//     @Override
//     public void runVolts(double volts) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'runVolts'");
//     }

//     @Override
//     public Command runVoltsCommand(double volts) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'runVoltsCommand'");
//     }

//     @Override
//     public void stop() {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'stop'");
//     }

//     @Override
//     public Command stopCommand() {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'stopCommand'");
//     }

//     @Override
//     public void initShuffleboard() {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'initShuffleboard'");
//     }

//     @Override
//     public void setNeutralMode(Neutral mode) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'setNeutralMode'");
//     }

//     @Override
//     public double getVolts() {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'getVolts'");
//     }
// }
