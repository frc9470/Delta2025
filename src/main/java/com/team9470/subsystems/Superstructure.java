package com.team9470.subsystems;

import com.team9470.Constants.ElevatorConstants;
import com.team9470.Constants.ArmConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

public class Superstructure extends SubsystemBase {
    private final Elevator elevator;
    private final Indexer indexer;
    private final Arm arm;
    private final Intake intake;
    private final Swerve drivetrain;

    private enum GamePieceState {
        EMPTY,
        CORAL_IN_CRADLE,
        CORAL_IN_ARM,
        ALGAE_IN_ARM
    }

    private enum Mode {
        NONE,
        CORAL,
        ALGAE
    }

    public enum Level {
        L1(1),
        L2(2),
        L3(3),
        L4(4);

        private final int elevatorLevel;

        Level(int elevatorLevel) {
            this.elevatorLevel = elevatorLevel;
        }

        public int elevatorLevel() {
            return elevatorLevel;
        }
    }

    private GamePieceState pieceState = GamePieceState.EMPTY;
    private Mode activeMode = Mode.NONE;
    public Level currentLevel = Level.L1;
    private Command autoPickupCommand;

    public Superstructure(Mechanism2d mech, Swerve drivetrain) {
        this.elevator = new Elevator(mech);
        this.indexer = new Indexer();
        this.arm = new Arm();
        this.intake = new Intake();
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // After the auto-pickup command is done, it is de-scheduled.
        if (autoPickupCommand != null && !autoPickupCommand.isScheduled()) {
            autoPickupCommand = null;
        }

        boolean coralInCradle = indexer.hasCoral();
        boolean newlyDetected = coralInCradle && pieceState == GamePieceState.EMPTY;
        boolean cradleCleared = !coralInCradle && pieceState == GamePieceState.CORAL_IN_CRADLE;
        if (coralInCradle)
            pieceState = GamePieceState.CORAL_IN_CRADLE;
        if(newlyDetected) {
            if (!arm.isHoldingItem() && autoPickupCommand == null) {
                autoPickupCommand = coralCradlePickup();
                CommandScheduler.getInstance().schedule(autoPickupCommand);
            }
        } else if (cradleCleared) {
            if (arm.isHoldingItem()) {
                pieceState = GamePieceState.CORAL_IN_ARM;
            }
        }

//        if (!arm.isHoldingItem()) {
//            if (pieceState == GamePieceState.CORAL_IN_ARM) {
//                pieceState = cradle ? GamePieceState.CORAL_IN_CRADLE : GamePieceState.EMPTY;
//                arm.setHoldingCoral(false);
//                activeMode = Mode.NONE;
//            } else if (pieceState == GamePieceState.ALGAE_IN_ARM) {
//                pieceState = cradle ? GamePieceState.CORAL_IN_CRADLE : GamePieceState.EMPTY;
//                arm.setHoldingAlgae(false);
//                activeMode = Mode.NONE;
//            }
//        }

        logTelemetry();
    }

    private void logTelemetry() {
        SmartDashboard.putString("Superstructure/GamePieceState", pieceState.name());
        SmartDashboard.putString("Superstructure/Mode", activeMode.name());
        SmartDashboard.putString("Superstructure/Level", currentLevel.name());

        // Logging current commands each subsystem is using
        Command currentArmCommand = arm.getCurrentCommand();
        if (currentArmCommand != null) {
            SmartDashboard.putString("Superstructure/Commands/Arm", arm.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("Superstructure/Commands/Arm", "null");
        }
    }

    public Command reverseCoral() {
        return indexer.reverseCommand();
    }

    public Command algaeUp() {
        return Commands.none();
    }

    public Command algaeDown() {
        return Commands.none();
    }

    public Command raise(int level) {
        return elevator.getLevelCommand(level);
    }

    public Command raise(Supplier<Integer> getLevel) {
        return elevator.getLevelCommand(getLevel.get());
    }

    public Command score() {
        return indexer.outputCommand();
    }

    public Command scoreSlow() {
        return indexer.outputCommand();
    }

    public Command triggerAlgaeHoming() {
        return Commands.none();
    }

    public Command waitForIntake() {
        return new WaitUntilCommand(indexer::hasCoral);
    }

    public Command funnelOut() {
        return Commands.none();
    }

    public Command climberAction() {
        return Commands.none();
    }

    public boolean hasGamePiece() {
        return arm.isHoldingItem() || indexer.hasCoral();
    }

    public Elevator getElevator() {
        return elevator;
    }

    public Indexer getIndexer() {
        return indexer;
    }

    public Arm getArm() {
        return arm;
    }

    public Intake getIntake() {
        return intake;
    }

    public Command scoreAndFunnel() {
        return indexer.outputCommand();
    }

    public boolean hasCoralInArm() {
        return pieceState == GamePieceState.CORAL_IN_ARM;
    }

    public boolean hasAlgaeInArm() {
        return pieceState == GamePieceState.ALGAE_IN_ARM;
    }

    public Command coralCradlePickup() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    arm.clearGamePieceFlags();
                    activeMode = Mode.NONE;
                }),
                new ParallelCommandGroup(
                        elevator.L1(),
                        arm.moveCommand(ArmConstants.CORAL_HANDOFF_PREP_ANGLE)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(ArmConstants.INTAKE_TIMEOUT.in(Seconds)),
                        new ParallelCommandGroup(
                                elevator.getMoveToPositionCommand(ElevatorConstants.HOME_POSITION),
                                Commands.startEnd(arm::startIntake, arm::stopRollers)
                        )
                ),
                elevator.L1(),
                Commands.runOnce(() -> {
                    if (arm.isHoldingItem()) {
                        arm.holdItem();
                        arm.setHoldingCoral(true);
                        pieceState = GamePieceState.CORAL_IN_ARM;
                        activeMode = Mode.CORAL;
                        currentLevel = Level.L1;
                    }
                }),
                new ParallelCommandGroup(
                        elevator.stow(),
                        arm.stowCommand()
                )
        ).withName("CoralCradlePickup");
    }

    public Command algaeGroundPickup() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    activeMode = Mode.NONE;
                    arm.clearGamePieceFlags();
                }),
                new ParallelDeadlineGroup(
                        Commands.waitUntil(arm::isHoldingItem).withTimeout(ArmConstants.INTAKE_TIMEOUT.in(Seconds)),
                        new ParallelCommandGroup(
                                elevator.L0(),
                                arm.moveCommand(ArmConstants.ALGAE_GROUND_INTAKE_ANGLE),
                                Commands.startEnd(arm::startIntake, arm::stopRollers)
                        )
                ),
                Commands.runOnce(() -> {
                    if (arm.isHoldingItem()) {
                        arm.holdItem();
                        arm.setHoldingAlgae(true);
                        pieceState = GamePieceState.ALGAE_IN_ARM;
                        activeMode = Mode.ALGAE;
                        currentLevel = Level.L1;
                    } else {
                        arm.stopRollers();
                        arm.clearGamePieceFlags();
                        pieceState = indexer.hasCoral()
                                ? GamePieceState.CORAL_IN_CRADLE
                                : GamePieceState.EMPTY;
                        activeMode = Mode.NONE;
                    }
                }),
                new ParallelCommandGroup(
                        elevator.L0(),
                        arm.moveCommand(ArmConstants.ALGAE_HOLD_ANGLE)
                )
        ).withName("AlgaeGroundPickup");
    }

    public Command algaeReefPickup(Level level) {
        if (level != Level.L2 && level != Level.L3) {
            return Commands.none();
        }
        return Commands.sequence(
                Commands.runOnce(() -> {
                    activeMode = Mode.NONE;
                    arm.clearGamePieceFlags();
                }),
                new ParallelDeadlineGroup(
                        Commands.waitUntil(arm::isHoldingItem).withTimeout(ArmConstants.INTAKE_TIMEOUT.in(Seconds)),
                        new ParallelCommandGroup(
                                elevator.getLevelCommand(level.elevatorLevel()),
                                arm.moveCommand(ArmConstants.ALGAE_REEF_INTAKE_ANGLE),
                                Commands.startEnd(arm::startIntake, arm::stopRollers)
                        )
                ),
                Commands.runOnce(() -> {
                    if (arm.isHoldingItem()) {
                        arm.holdItem();
                        arm.setHoldingAlgae(true);
                        pieceState = GamePieceState.ALGAE_IN_ARM;
                        activeMode = Mode.ALGAE;
                        currentLevel = level;
                    } else {
                        arm.stopRollers();
                        arm.clearGamePieceFlags();
                        pieceState = indexer.hasCoral()
                                ? GamePieceState.CORAL_IN_CRADLE
                                : GamePieceState.EMPTY;
                        activeMode = Mode.NONE;
                    }
                }),
                new ParallelCommandGroup(
                        elevator.stow(),
                        arm.moveCommand(ArmConstants.ALGAE_HOLD_ANGLE)
                )
        ).withName("AlgaeReefPickup" + level.name());
    }

    public Command prepareLevel(Level level) {
        return Commands.defer(() -> switch (pieceState) {
            case CORAL_IN_ARM -> coralPrepare(level);
            case ALGAE_IN_ARM -> algaePrepare(level);
            case CORAL_IN_CRADLE, EMPTY -> {
                if (level == Level.L1)
                    yield algaeGroundPickup();
                else if (level == Level.L2 || level == Level.L3) {
                    yield algaeReefPickup(level);
                }
                yield Commands.none();
            }
        }, Set.of(this));
    }

    public Command scoreHeldPiece() {
        return Commands.defer(() -> switch (activeMode) {
            case CORAL -> coralScore(currentLevel);
            case ALGAE -> algaeScore(currentLevel);
            default -> Commands.none();
        }, Set.of(this));
    }

    public Command stow() {
        return new ParallelCommandGroup(
                elevator.stow(),
                arm.stowCommand()
        ).withName("SuperstructureStow");
    }

    private Command coralPrepare(Level level) {
        Command armCommand = switch (level) {
            case L4 -> arm.moveCommand(ArmConstants.CORAL_L4_BEFORE_SCORING);
            case L3 -> arm.moveCommand(ArmConstants.CORAL_L3_BEFORE_SCORING);
            case L2 -> arm.moveCommand(ArmConstants.CORAL_L2_BEFORE_SCORING);
            case L1 -> arm.moveCommand(ArmConstants.CORAL_L1_BEFORE_SCORING);
        };

        return new ParallelCommandGroup(
                elevator.getLevelCommand(level.elevatorLevel()),
                armCommand
        ).andThen(Commands.runOnce(() -> {
            activeMode = Mode.CORAL;
            currentLevel = level;
        }));
    }

    public Command coralScore(Level level) {
        Command armCommand = switch (level) {
            case L4 -> arm.moveCommand(ArmConstants.CORAL_L4_SCORING);
            case L3 -> arm.moveCommand(ArmConstants.CORAL_L3_SCORING);
            case L2 -> arm.moveCommand(ArmConstants.CORAL_L2_SCORING);
            case L1 -> arm.moveCommand(ArmConstants.CORAL_L1_SCORING);
        };

        Angle releaseAngle = switch (level) {
            case L4 -> ArmConstants.CORAL_L4_SCORING;
            case L3 -> ArmConstants.CORAL_L3_RELEASE;
            case L2 -> ArmConstants.CORAL_L2_RELEASE;
            case L1 -> ArmConstants.CORAL_L1_SCORING;
        };

        return Commands.sequence(
                new ParallelCommandGroup(
                        armCommand,
                        Commands.sequence(
                                new WaitUntilCommand(() -> arm.getAngle().isNear(releaseAngle, Degrees.of(3)))
                                        .withTimeout(2.0),
                                new ParallelCommandGroup(
                                        new RunCommand(arm::startOutput),
                                        this.run(() -> drivetrain.setChassisSpeeds(new ChassisSpeeds(-0.5, 0.0, 0.0)))
                                ).withTimeout(1).finallyDo(() -> drivetrain.setChassisSpeeds(new ChassisSpeeds()))
                        )
                ),
                Commands.runOnce(() -> {
                    arm.setHoldingCoral(false);
                    arm.stopRollers();
                    arm.clearGamePieceFlags();
                    pieceState = indexer.hasCoral()
                            ? GamePieceState.CORAL_IN_CRADLE
                            : GamePieceState.EMPTY;
                    activeMode = Mode.NONE;
                }),
                stow()
        ).withName("CoralScore" + level.name());
    }

    private Command algaePrepare(Level level) {
        Command armCommand = switch (level) {
            case L4 -> arm.algaeBargeBeforeScoringCommand();
            case L3, L2 -> arm.moveCommand(ArmConstants.ALGAE_REEF_INTAKE_ANGLE);
            case L1 -> arm.moveCommand(ArmConstants.ALGAE_PROCESSOR_BEFORE_SCORING);
        };

        Command elevatorCommand = switch (level) {
            case L4 -> elevator.L4();
            case L3 -> elevator.L3();
            case L2 -> elevator.L2();
            case L1 -> elevator.L1();
        };

        return new ParallelCommandGroup(
                elevatorCommand,
                armCommand
        ).andThen(Commands.runOnce(() -> {
            activeMode = Mode.ALGAE;
            currentLevel = level;
        }));
    }

    private Command algaeScore(Level level) {
        return switch (level) {
            case L4 -> Commands.sequence(
                    elevator.L4(),
                    arm.algaeBargeScoringCommand(),
                    finalizeAlgaeScore()
            ).withName("AlgaeScoreBarge");
            case L1 -> Commands.sequence(
                    elevator.L0(),
                    arm.algaeProcessorScoringCommand(),
                    finalizeAlgaeScore()
            ).withName("AlgaeScoreProcessor");
            case L3, L2 -> Commands.none();
        };
    }

    private Command finalizeAlgaeScore() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    arm.setHoldingAlgae(false);
                    arm.stopRollers();
                    arm.clearGamePieceFlags();
                    pieceState = indexer.hasCoral()
                            ? GamePieceState.CORAL_IN_CRADLE
                            : GamePieceState.EMPTY;
                    activeMode = Mode.NONE;
                }),
                stow()
        );
    }

    // Legacy scoring helpers retained for compatibility with autonomous code.

    public Command scoreAlgaeBarge() {
        return algaePrepare(Level.L4).andThen(algaeScore(Level.L4));
    }

    public Command scoreAlgaeProcessor() {
        return algaePrepare(Level.L1).andThen(algaeScore(Level.L1));
    }


    // TODO: Remove after debugging.
    public Command intakeCoralGround() {
        return Commands.sequence(
                new ParallelDeadlineGroup(
                        Commands.waitUntil(indexer::hasCoral),
                        new ParallelCommandGroup(
                                intake.goDownAndStartRollersCommand(),
                                indexer.runCommand()
                        )
                ),
                stopIntakeGround()
        );
    }

    public Command stopIntakeGround() {
        return intake.retractAndStopRollersCommand();
    }

    public Command intakeAlgaeGround() {
        return algaeGroundPickup();
    }

    public Command intakeAlgaeReef() {
        return algaeReefPickup(Level.L2);
    }

    /**
     * Debug helper that bypasses the superstructure state machine and directly moves the arm.
     */
    public Command debugArmToAngle(Angle angle) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    activeMode = Mode.NONE;
                }),
                arm.moveCommand(angle)
        ).withName("DebugArmTo" + angle);
    }

    /**
     * Debug helper that bypasses the superstructure state machine and directly moves the elevator.
     */
    public Command debugElevatorToHeight(Distance height) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    activeMode = Mode.NONE;
                }),
                elevator.getMoveToPositionCommand(height)
        ).withName("DebugElevatorTo" + height);
    }

    /**
     * Debug helper to position both the arm and elevator simultaneously.
     */
    public Command debugPose(Distance height, Angle angle) {
        return new ParallelCommandGroup(
                debugElevatorToHeight(height),
                debugArmToAngle(angle)
        ).withName("DebugPose");
    }

    // TODO: Remove after testing.
    public Command moveArmToAngle(Angle angle) {
        return arm.moveCommand(angle);
    }

    public Command moveElevatorToHeight(Distance height) {
        return elevator.getMoveToPositionCommand(height);
    }

    public Command armIntake() {
        return arm.runIntakeCommand();
    }

    public Command armOuttake() {
        return arm.runOuttakeCommand();
    }

    public Command armStop() {
        return arm.stopRollersCommand();
    }
}
