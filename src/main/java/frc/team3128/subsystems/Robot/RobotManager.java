package frc.team3128.subsystems.Robot;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Climber.Climber;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.Elevator;
import frc.team3128.subsystems.Elevator.ElevatorMechanism;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.Intake;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.Manipulator;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import edu.wpi.first.math.Pair;

public class RobotManager extends FSMSubsystemBase<RobotStates> {
    private static RobotManager instance;
    private static Elevator elevator;
    private static Manipulator manipulator;
    private static Intake intake;
    private static Climber climber;
    private static Swerve swerve;

    private static TransitionMap<RobotStates> transitionMap = new TransitionMap<>(RobotStates.class);
    private Function<RobotStates, Command> defaultTransitioner = state -> {return updateSubsystemStates(state);};

    private RobotManager() {
        super(RobotStates.class, transitionMap, NEUTRAL);

        elevator = Elevator.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
        swerve = Swerve.getInstance();

        initShuffleboard();
        NAR_Shuffleboard.addData(this.getName(), "Auto Enabled", () -> Swerve.autoEnabled);
    }

    public static synchronized RobotManager getInstance() {
        if (instance == null) {
            instance = new RobotManager();
        }

        return instance;
    }

    public Command updateSubsystemStates(RobotStates nextState) {
        return sequence(
            waitUntil(()-> (!Swerve.autoEnabled)).onlyIf(()-> nextState.getWaitForAutoEnabled()),
            elevator.setStateCommand(nextState.getElevatorState()).unless(()-> nextState.getElevatorState() == ElevatorStates.UNDEFINED),
            manipulator.setStateCommand(nextState.getManipulatorState()).unless(()-> nextState.getManipulatorState() == ManipulatorStates.UNDEFINED),
            intake.setStateCommand(nextState.getIntakeState()).unless(()-> nextState.getIntakeState() == IntakeStates.UNDEFINED),
            climber.setStateCommand(nextState.getClimberState()).unless(()-> nextState.getClimberState() == ClimberStates.UNDEFINED),
            waitUntil(()-> climber.winch.atSetpoint()).unless(()-> nextState.getClimberState() == ClimberStates.UNDEFINED),
            runOnce(()-> swerve.setThrottle(nextState.getThrottle())).unless(()-> DriverStation.isAutonomous() || Swerve.autoEnabled)
        );
    }

    public void autoScore() {
        if (getState() == RobotStates.NEUTRAL || getState() == RobotStates.FULL_NEUTRAL)
            return;

        
        if(getState() == AUTO_HOLD || getState() == AUTO_HOLD2) {
            setStateCommand(NEUTRAL).schedule();
            return;
        }


        coupledStates.forEach((Pair<RobotStates, RobotStates> coupledState) -> {
            if (coupledState.getFirst() == getState()) {
                sequence(
                    waitUntil(() -> ElevatorMechanism.getInstance().atSetpoint()),
                    setStateCommand(coupledState.getSecond()),
                    waitSeconds(0.5),
                    setStateCommand(NEUTRAL)
                ).schedule();
            }
        });
    }

    public Command getTempToggleCommand(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition, double delay) {
        return either(
            sequence(
                setStateCommand(exclusiveState),
                waitSeconds(0.5),
                setStateCommand(NEUTRAL)
            ),
            setStateCommand(defaultState), 
            condition
        );
    }

    public Command getTempToggleCommand(RobotStates defaultStates, RobotStates exclusiveState, BooleanSupplier condition) {
        return getTempToggleCommand(defaultStates, exclusiveState, condition, 1);
    }

    public Command getTempToggleCommand(RobotStates defaultStates, RobotStates exclusiveState) {
        return getTempToggleCommand(defaultStates, exclusiveState,  ()-> stateEquals(defaultStates), 1);
    }

    public Command getToggleCommand(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        return either(
            setStateCommand(exclusiveState), 
            setStateCommand(defaultState), 
            condition
        );
    }

    public Command getToggleCommand(RobotStates defaultState, RobotStates exclusiveState) {
        return getToggleCommand(defaultState, exclusiveState, ()-> stateEquals(defaultState));
    }

    public Command getToggleCommand(RobotStates state) {
        return getToggleCommand(state, NEUTRAL);
    }

    @Override
    public void registerTransitions() {

        // Between all default states
        transitionMap.addCommutativeTransition(defaultStates.asJava(), defaultTransitioner);

        // For each coupled states pair
        transitionMap.addMappedTransition(coupledStates.asJava(), defaultTransitioner);

        // From exclusive state to Neutral
        transitionMap.addConvergingTransition(exclusiveStates.asJava(), NEUTRAL, defaultTransitioner);

        transitionMap.addTransition(RPL1, RSL1, sequence(
            waitUntil(()-> !Swerve.autoEnabled),
            runOnce(()-> swerve.setThrottle(RSL1.getThrottle())).unless(()-> DriverStation.isAutonomous()),
            manipulator.setStateCommand(RSL1.getManipulatorState()),
            waitSeconds(0.25),
            elevator.setStateCommand(RSL1.getElevatorState())
        ));

    }
}
