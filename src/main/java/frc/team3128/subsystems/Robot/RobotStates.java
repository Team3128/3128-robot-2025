package frc.team3128.subsystems.Robot;

import static frc.team3128.subsystems.Intake.IntakeStates.INTAKE;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;
import io.vavr.collection.List;
import static frc.team3128.Constants.DriveConstants.slow;
import static frc.team3128.Constants.DriveConstants.fast;

public enum RobotStates {
    
    FULL_NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.NEUTRAL),
    NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.UNDEFINED),

    RPL1(ElevatorStates.LOW_L1, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true),
    RPL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true),
    RPL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true), 
    RPL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, slow, true),
    
    RSL1(ElevatorStates.HIGH_L1, IntakeStates.NEUTRAL, ManipulatorStates.OUT_L1, slow, true),
    RSL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true),
    RSL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true),
    RSL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.OUT, slow, true),

    AUTO_HOLD(ElevatorStates.AUTO_HOLD, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, fast, false),
    AUTO_HOLD2(ElevatorStates.AUTO_HOLD2, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, fast, false),
    
    INTAKE(ElevatorStates.NEUTRAL, IntakeStates.INTAKE, ManipulatorStates.NEUTRAL),
    EJECT_OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.EJECT_OUTTAKE, ManipulatorStates.NEUTRAL),
    HIGH_INTAKE(ElevatorStates.NEUTRAL, IntakeStates.HIGH_INTAKE, ManipulatorStates.NEUTRAL),
    
    PRE_CLIMB_PRIME(ClimberStates.PRE_CLIMB_PRIME),
    CLIMB_PRIME(ElevatorStates.NEUTRAL, IntakeStates.CLIMB_PRIME, ManipulatorStates.NEUTRAL, ClimberStates.CLIMB_PRIME, slow),
    CLIMB(ElevatorStates.NEUTRAL, IntakeStates.CLIMB, ManipulatorStates.NEUTRAL, ClimberStates.CLIMB, slow);

    
    
    
    
    
    
    
    private ElevatorStates elevator;
    private IntakeStates intake;
    private ManipulatorStates manipulator;
    private ClimberStates climber;
    private double throttle;
    private boolean waitForAutoEnabled;

    public static final List<RobotStates> defaultElevatorStates = List.of(RPL1, RPL2, RPL3, RPL4, AUTO_HOLD, AUTO_HOLD2);
    public static final List<RobotStates> exclusiveElevatorStates = List.of(RSL1, RSL2, RSL3, RSL4);
    public static final List<RobotStates> defaultIntakeStates = List.of(INTAKE, EJECT_OUTTAKE, HIGH_INTAKE);
    public static final List<RobotStates> defaultClimbStates = List.of(CLIMB_PRIME, PRE_CLIMB_PRIME);
    public static final List<RobotStates> exclusiveClimbStates = List.of(CLIMB);

    public static final List<Pair<RobotStates, RobotStates>> coupledStates = List.of(
        // Pair.of(RPL1, RSL1),
        Pair.of(RPL2, RSL2),
        Pair.of(RPL3, RSL3),
        Pair.of(RPL4, RSL4),
        Pair.of(CLIMB_PRIME, CLIMB)
    );

    public static final List<RobotStates> defaultStates = List.of(NEUTRAL, FULL_NEUTRAL).appendAll(defaultElevatorStates).appendAll(defaultIntakeStates).appendAll(defaultClimbStates);
    public static final List<RobotStates> exclusiveStates = exclusiveElevatorStates.appendAll(exclusiveClimbStates);

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, double throttle, boolean waitForAutoEnabled) {
        this.elevator = elevator;
        this.intake = intake;
        this.manipulator = manipulator;
        this.climber = climber;
        this.throttle = throttle;
        this.waitForAutoEnabled = waitForAutoEnabled;
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, double throttle) {
        this(elevator, intake, manipulator, climber, throttle, false);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber) {
        this(elevator, intake, manipulator, climber, 1, false);
    }


    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, double throttle) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, throttle, false);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, 1, false);
    }

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, double throttle, boolean waitForAutoEnabled) {
        this(elevator, intake, manipulator, ClimberStates.UNDEFINED, throttle, waitForAutoEnabled);
    }

    private RobotStates(ClimberStates climber, ElevatorStates elevator, double throttle) {
        this(elevator, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, climber, throttle, false);
    }

    private RobotStates(ClimberStates climber) {
        this(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, climber, fast, false);
    }

    private RobotStates() {
        this(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, ClimberStates.UNDEFINED);
    }

    public ElevatorStates getElevatorState() {
        return this.elevator;
    }

    public IntakeStates getIntakeState() {
        return this.intake;
    }

    public ManipulatorStates getManipulatorState() {
        return this.manipulator;
    }

    public ClimberStates getClimberState() {
        return this.climber;
    }

    public double getThrottle() {
        return this.throttle;
    }

    public boolean getWaitForAutoEnabled() {
        return this.waitForAutoEnabled;
    }
}