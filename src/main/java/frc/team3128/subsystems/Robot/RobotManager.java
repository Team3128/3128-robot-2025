package frc.team3128.subsystems.Robot;

import common.core.fsm.FSMSubsystemBase;
import common.core.fsm.TransitionMap;
import common.utility.Log;
import common.utility.shuffleboard.NAR_Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands.*;
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
import frc.team3128.RobotContainer;
import frc.team3128.Constants.FieldConstants.FieldStates;
import static frc.team3128.Constants.FieldConstants.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.team3128.subsystems.Robot.RobotStates.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotManager extends FSMSubsystemBase<RobotStates> {
    private static RobotManager instance;
    private static Elevator elevator;
    private static Manipulator manipulator;
    private static Intake intake;
    private static Climber climber;
    private static Swerve swerve;

    private static TransitionMap<RobotStates> transitionMap = new TransitionMap<>(RobotStates.class);
    private Function<RobotStates, Command> defaultTransitioner = state -> {return updateSubsystemStates(state);};

    private static boolean delayTransition = false;

    private RobotManager() {
        super(RobotStates.class, transitionMap, FULL_NEUTRAL);

        elevator = Elevator.getInstance();
        manipulator = Manipulator.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
        swerve = Swerve.getInstance();

        initShuffleboard();
        NAR_Shuffleboard.addData(this.getName(), "Auto Enabled", () -> Swerve.autoMoveEnabled);
        NAR_Shuffleboard.addData("Auto", "DelayTransition", () -> delayTransition, 3, 3);

        registerTransitions();
    }

    public static synchronized RobotManager getInstance() {
        if (instance == null) {
            instance = new RobotManager();
        }

        return instance;
    }

    public Command updateSubsystemStates(RobotStates nextState) {
        return sequence(
            waitUntil(()-> (!delayTransition)).onlyIf(()-> nextState.getDelayTransition()), //Pauses transition until transitions are unpaused
            elevator.setStateCommand(nextState.getElevatorState()).unless(()-> nextState.getElevatorState() == ElevatorStates.UNDEFINED),
            manipulator.setStateCommand(nextState.getManipulatorState()).unless(()-> nextState.getManipulatorState() == ManipulatorStates.UNDEFINED),
            intake.setStateCommand(nextState.getIntakeState()).unless(()-> nextState.getIntakeState() == IntakeStates.UNDEFINED),
            climber.setStateCommand(nextState.getClimberState()).unless(()-> nextState.getClimberState() == ClimberStates.UNDEFINED),
            waitUntil(()-> climber.winch.atSetpoint()).unless(()-> nextState.getClimberState() == ClimberStates.UNDEFINED),
            runOnce(()-> swerve.setThrottle(nextState.getThrottle())).unless(()-> DriverStation.isAutonomous() || Swerve.autoMoveEnabled)
        );
    }
    public Command alignScoreCoral(boolean isRight){
        final List<Pose2d> setpoints = isRight ? FieldStates.reefRight.asJava() : FieldStates.reefLeft.asJava();
        Supplier<Pose2d> pose = () -> swerve.nearestPose2d(allianceFlip(setpoints));
        return alignScoreCoral(pose, ()-> false, ()->true);
    }

    public Command alignScoreCoral(boolean isRight, BooleanSupplier shouldWait){
        final List<Pose2d> setpoints = isRight ? FieldStates.reefRight.asJava() : FieldStates.reefLeft.asJava();
        Supplier<Pose2d> pose = () -> swerve.nearestPose2d(allianceFlip(setpoints));
        return alignScoreCoral(pose, ()-> false, shouldWait);
    }

    public Command alignScoreCoral(Supplier<Pose2d> pose, BooleanSupplier shouldRam) {
        return alignScoreCoral(pose, shouldRam, ()->false);
    }

    public Command alignScoreCoralSlow(Supplier<Pose2d> pose, BooleanSupplier shouldRam) {
        return alignScoreCoral(pose, shouldRam, ()->true);
    }

    public Command alignScoreCoral(Supplier<Pose2d> pose, BooleanSupplier shouldRam, BooleanSupplier shouldWait){
        return parallel(
            sequence(
                Commands.runOnce(()-> delayTransition = true),
                swerve.navigateTo(pose),
                swerve.ram(pose).onlyIf(shouldRam)
            ),
            sequence(
                waitUntil(()-> swerve.atElevatorDist()), // wait until safe for elevator to move
                Commands.runOnce(()-> delayTransition = false),
                Commands.runOnce(()-> {
                    for(Pair<RobotStates, RobotStates> coupledState : coupledStates){
                        if (coupledState.getFirst() == getState()) {
                            sequence(
                                waitUntil(() -> ElevatorMechanism.getInstance().atSetpoint()),
                                waitUntil(()-> !Swerve.autoMoveEnabled).onlyIf(shouldWait),
                                setStateCommand(coupledState.getSecond()),
                                waitSeconds(0.5),
                                setStateCommand(NEUTRAL)
                            ).schedule();
                            return;
                        }
                    }
                })
            )
        );
    }

    public Command alignAlgaeIntake() {
        final List<Pose2d> setpoints = FieldStates.algaePoses.asJava();
        Supplier<Pose2d> pose = ()-> swerve.nearestPose2d(allianceFlip(setpoints));
        return alignAlgaeIntake(pose);
    }

    public Command alignAlgaeIntake(Supplier<Pose2d> pose) {
        return parallel(
            swerve.navigateTo(pose),
            Commands.runOnce(
                ()-> {
                    if(FieldStates.idOf(allianceFlip(pose).get()) % 2 == 0) setStateCommand(RSA2).schedule();
                    else setStateCommand(RSA1).schedule();
                }
            )
        );
    }

    public Command alignCoralIntake() {
        final List<Pose2d> setpoints = List.of(FieldStates.SOURCE_LEFT.getPose2d(), FieldStates.SOURCE_RIGHT.getPose2d());
        Supplier<Pose2d> pose = ()-> swerve.nearestPose2d(allianceFlip(setpoints));
        return parallel(
            swerve.navigateTo(pose),
            setStateCommand(NEUTRAL)
        );
    }

    public Command alignAlgaeScore() {
        Supplier<Pose2d> pose = ()-> allianceFlip(new Pose2d(new Translation2d(7.6, swerve.getPose().getY()), Rotation2d.fromDegrees(0)));
        return alignAlgaeScore(pose);
    }
 
    public Command alignAlgaeScore(Supplier<Pose2d> pose) {
        return parallel(
            swerve.navigateTo(pose),
            sequence(
                waitUntil(()-> swerve.atElevatorDist()), // wait until safe for elevator to move
                setStateCommand(RPB),
                Commands.runOnce(()-> delayTransition = false),
                waitUntil(() -> ElevatorMechanism.getInstance().atSetpoint()),
                waitUntil(()-> !Swerve.autoMoveEnabled),
                setStateCommand(RSB),
                waitSeconds(0.5),
                setStateCommand(NEUTRAL)
            )
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

    public Command getToggleCommand(Command defaultCommand, Command exclusiveCommand, BooleanSupplier condition) {
        return either(
            exclusiveCommand,
            defaultCommand,
            condition
        );
    }

    public Command getToggleCommand(RobotStates defaultState, RobotStates exclusiveState, BooleanSupplier condition) {
        return getToggleCommand(setStateCommand(defaultState), setStateCommand(exclusiveState), condition);
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
        transitionMap.addConvergingTransition(FULL_NEUTRAL, defaultTransitioner);
        transitionMap.addDivergingTransition(FULL_NEUTRAL, defaultTransitioner);

        transitionMap.addTransition(RPL1, RSL1, sequence(
            waitUntil(()-> !Swerve.autoMoveEnabled),
            runOnce(()-> swerve.setThrottle(RSL1.getThrottle())).unless(()-> DriverStation.isAutonomous()),
            manipulator.setStateCommand(RSL1.getManipulatorState()),
            waitSeconds(0.25),
            elevator.setStateCommand(RSL1.getElevatorState())
        ));
    }
}
