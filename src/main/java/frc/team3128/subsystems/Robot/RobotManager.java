package frc.team3128.subsystems.Robot;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

import io.vavr.collection.List;

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

    public static boolean pauseTransitions = false;

    final List<Subsystem> subsystemList = List.of(elevator, manipulator, intake, climber, swerve);

    private static TransitionMap<RobotStates> transitionMap = new TransitionMap<>(RobotStates.class);
    private Function<RobotStates, Command> discreteTransitioner = state -> {return discreteUpdateStates(state, subsystemList);};

    private RobotManager() {
        super(RobotStates.class, transitionMap, NEUTRAL);

        elevator = Elevator.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
        swerve = Swerve.getInstance();

        initShuffleboard();
        NAR_Shuffleboard.addData(this.getName(), "Auto Enabled", () -> Swerve.autoMoveEnabled);

        registerTransitions();
    }

    public static synchronized RobotManager getInstance() {
        if (instance == null) {
            instance = new RobotManager();
        }

        return instance;
    }

    public Command discreteUpdateStates(RobotStates nextState, Command command, Subsystem... updateSubsystems) {
        return discreteUpdateStates(nextState, List.of(updateSubsystems));
    }

    public Command discreteUpdateStates(RobotStates nextState, List<Subsystem> updateSubsystems) {
        return parallel(
            either(elevator.setStateCommand(nextState.getElevatorState()).unless(()-> nextState.getElevatorState() == ElevatorStates.UNDEFINED), Commands.none(), () -> updateSubsystems.contains(elevator)),
            either(manipulator.setStateCommand(nextState.getManipulatorState()).unless(()-> nextState.getManipulatorState() == ManipulatorStates.UNDEFINED), Commands.none(), () -> updateSubsystems.contains(manipulator)),
            either(intake.setStateCommand(nextState.getIntakeState()).unless(()-> nextState.getIntakeState() == IntakeStates.UNDEFINED), Commands.none(), () -> updateSubsystems.contains(intake)),
            either(climber.setStateCommand(nextState.getClimberState()).unless(()-> nextState.getClimberState() == ClimberStates.UNDEFINED), Commands.none(), () -> updateSubsystems.contains(climber)),
            either(runOnce(()-> swerve.setThrottle(nextState.getThrottle())).unless(()-> DriverStation.isAutonomous() || Swerve.autoMoveEnabled), Commands.none(), () -> updateSubsystems.contains(swerve))
        );
    }

    public Command indiscreteUpdateStates(RobotStates nextState, Command command, Subsystem... updateSubsystems) {
        return indiscreteUpdateStates(nextState, command, List.of(updateSubsystems));
    }

    public Command indiscreteUpdateStates(RobotStates nextState, Command command, List<Subsystem> updateSubsystems) {
        return parallel(
            discreteUpdateStates(nextState, subsystemList.removeAll(updateSubsystems)),
            command
        );
    }

    public void autoScore() {
        if (getState() == RobotStates.NEUTRAL || getState() == RobotStates.FULL_NEUTRAL)
            return;

        
        if(getState() == TELE_HOLD || getState() == AUTO_HOLD) {
            setStateCommand(NEUTRAL).schedule();
            return;
        }


        coupledStates.forEach((Pair<RobotStates, RobotStates> coupledState) -> {
            if (coupledState.getFirst() == getState()) {
                sequence(
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
        transitionMap.addCommutativeTransition(defaultStates.asJava(), discreteTransitioner);

        // For each coupled states pair
        transitionMap.addMappedTransition(coupledStates.asJava(), discreteTransitioner);

        // From exclusive state to Neutral
        transitionMap.addConvergingTransition(exclusiveStates.asJava(), NEUTRAL, discreteTransitioner);

        transitionMap.addConvergingTransition(
            defaultStates.asJava(), 
            RSL1, 
            indiscreteUpdateStates(
                RSL1, 
                sequence(
                    waitUntil(()-> Swerve.autoMoveEnabled),
                    elevator.setStateCommand(RPL1.getElevatorState()),
                    waitUntil(()-> elevator.elevator.atSetpoint()),
                    manipulator.setStateCommand(RSL1.getManipulatorState()),
                    waitSeconds(0.25),
                    elevator.setStateCommand(RSL1.getElevatorState())
                ), 
                elevator, manipulator
            )
        );

        transitionMap.addConvergingTransition(
            defaultStates.asJava(), 
            List.of(RSL2, RSL3, RSL4).asJava(), 
            (RobotStates nextState)-> {
                return indiscreteUpdateStates(
                    nextState, 
                    sequence(
                        waitUntil(()-> !pauseTransitions),
                        manipulator.setStateCommand(ManipulatorStates.NEUTRAL),
                        elevator.setStateCommand(nextState.getElevatorState()),
                        waitUntil(()-> elevator.elevator.atSetpoint()),
                        manipulator.setStateCommand(nextState.getManipulatorState())
                    ), 
                    elevator, manipulator
                );
            } 
        );

        transitionMap.addConvergingTransition(
            defaultStates.asJava(),
            CLIMB_PRIME,
            indiscreteUpdateStates(
                CLIMB_PRIME, 
                sequence(
                    climber.setStateCommand(CLIMB_PRIME.getClimberState()),
                    waitUntil(()-> climber.winch.atSetpoint()),
                    runOnce(()-> swerve.setThrottle(CLIMB_PRIME.getThrottle()))
                ), 
                climber, swerve
            )
        );

        transitionMap.addConvergingTransition(
            defaultStates.asJava(),
            RPL4,
            indiscreteUpdateStates(
                RPL4, 
                sequence(
                    waitUntil(()-> !pauseTransitions),
                    elevator.setStateCommand(RPL4.getElevatorState())
                ), 
                elevator)
        );

        transitionMap.addTransition(
            CLIMB,
            CLIMB_PRIME,
            discreteTransitioner
        );

    }
}
