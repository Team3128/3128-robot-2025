package frc.team3128.subsystems.Robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import frc.team3128.subsystems.Climber.ClimberStates;
import frc.team3128.subsystems.Elevator.ElevatorStates;
import frc.team3128.subsystems.Intake.IntakeStates;
import frc.team3128.subsystems.Manipulator.ManipulatorStates;
import frc.team3128.subsystems.Swerve.SwerveStates;
import io.vavr.collection.List;

public enum RobotStates {
    
    FULL_NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.NEUTRAL, SwerveStates.NEUTRAL),
    NEUTRAL(ElevatorStates.NEUTRAL, IntakeStates.NEUTRAL, ManipulatorStates.IN, ClimberStates.UNDEFINED, SwerveStates.NEUTRAL),

    RPL1(ElevatorStates.LOW_L1, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.SCORING),
    RPL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.SCORING),
    RPL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.SCORING), 
    RPL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.SCORING),
    
    RSL1(ElevatorStates.HIGH_L1, IntakeStates.NEUTRAL, ManipulatorStates.OUT_L1, ClimberStates.UNDEFINED, SwerveStates.SCORING),
    RSL2(ElevatorStates.L2, IntakeStates.NEUTRAL, ManipulatorStates.OUT, ClimberStates.UNDEFINED, SwerveStates.SCORING),
    RSL3(ElevatorStates.L3, IntakeStates.NEUTRAL, ManipulatorStates.OUT, ClimberStates.UNDEFINED, SwerveStates.SCORING),
    RSL4(ElevatorStates.L4, IntakeStates.NEUTRAL, ManipulatorStates.OUT, ClimberStates.UNDEFINED, SwerveStates.SCORING),

    AUTO_HOLD(ElevatorStates.AUTO_HOLD, IntakeStates.NEUTRAL, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.AUTO_MOVEMENT),
    
    INTAKE(ElevatorStates.NEUTRAL, IntakeStates.INTAKE, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.NEUTRAL),
    EJECT_OUTTAKE(ElevatorStates.NEUTRAL, IntakeStates.EJECT_OUTTAKE, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.NEUTRAL),
    HIGH_INTAKE(ElevatorStates.NEUTRAL, IntakeStates.HIGH_INTAKE, ManipulatorStates.NEUTRAL, ClimberStates.UNDEFINED, SwerveStates.NEUTRAL),
    
    PRE_CLIMB_PRIME(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, ClimberStates.PRE_CLIMB_PRIME, SwerveStates.UNDEFINED),
    CLIMB_PRIME(ElevatorStates.NEUTRAL, IntakeStates.CLIMB_PRIME, ManipulatorStates.UNDEFINED, ClimberStates.CLIMB_PRIME, SwerveStates.CLIMBING),
    CLIMB(ElevatorStates.NEUTRAL, IntakeStates.CLIMB, ManipulatorStates.NEUTRAL, ClimberStates.CLIMB, SwerveStates.CLIMBING);

    
    
    
    
    
    
    
    private ElevatorStates elevator;
    private IntakeStates intake;
    private ManipulatorStates manipulator;
    private ClimberStates climber;
    private SwerveStates swerve;

    public static final List<RobotStates> defaultElevatorStates = List.of(RPL1, RPL2, RPL3, RPL4, AUTO_HOLD);
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

    private RobotStates(ElevatorStates elevator, IntakeStates intake, ManipulatorStates manipulator, ClimberStates climber, SwerveStates swerve) {
        this.elevator = elevator;
        this.intake = intake;
        this.manipulator = manipulator;
        this.climber = climber;
        this.swerve = swerve;
    }


    private RobotStates() {
        this(ElevatorStates.UNDEFINED, IntakeStates.UNDEFINED, ManipulatorStates.UNDEFINED, ClimberStates.UNDEFINED, SwerveStates.UNDEFINED);
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

    public SwerveStates getSwerveState() {
        return this.swerve;
    }

}