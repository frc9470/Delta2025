package com.team9470.subsystems;

import com.team9470.subsystems.arm.Arm;
import com.team9470.subsystems.arm.ArmConstants;
import com.team9470.subsystems.elevator.Elevator;
import com.team9470.subsystems.elevator.ElevatorConstants;
import com.team9470.subsystems.indexer.Indexer;
import com.team9470.subsystems.intake.Intake;
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

    private enum PieceType {
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

    /** Physical game piece currently clamped inside the arm's rollers. */
    private PieceType heldPiece = PieceType.NONE;
    /** Whether the indexer is staging a coral in the handoff cradle for pickup. */
    private boolean coralInCradle = false;
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

        boolean cradleHasCoral = indexer.hasCoral();
        boolean cradleStateChanged = cradleHasCoral != coralInCradle;
        if (cradleStateChanged) {
            coralInCradle = cradleHasCoral;
            if (!cradleHasCoral && arm.isHoldingItem() && heldPiece == PieceType.NONE) {
                heldPiece = PieceType.CORAL;
            }
        }

        if (coralInCradle && heldPiece == PieceType.NONE && !arm.isHoldingItem() && autoPickupCommand == null) {
            autoPickupCommand = coralCradlePickup();
            CommandScheduler.getInstance().schedule(autoPickupCommand);

        }

        logTelemetry();
    }

    private void logTelemetry() {
        SmartDashboard.putString("Superstructure/HeldPiece", heldPiece.name());
        SmartDashboard.putBoolean("Superstructure/CoralInCradle", coralInCradle);
        SmartDashboard.putString("Superstructure/Level", currentLevel.name());

        if (arm.getCurrentCommand() != null) {
            SmartDashboard.putString("Superstructure/ARMCommand", arm.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("Superstructure/ARMCommand", "null");
        }

        // Logging current commands each subsystem is using
        if (autoPickupCommand != null) {
            SmartDashboard.putString("Superstructure/AutoPickupCommand", autoPickupCommand.getName());
        } else {
            SmartDashboard.putString("Superstructure/AutoPickupCommand", "null");
        }
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

    public Command waitForIntake() {
        return new WaitUntilCommand(indexer::hasCoral);
    }

    public boolean hasGamePiece() {
        return heldPiece != PieceType.NONE || coralInCradle;
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
        return heldPiece == PieceType.CORAL;
    }

    public boolean hasAlgaeInArm() {
        return heldPiece == PieceType.ALGAE;
    }

    public Command coralCradlePickup() {
        return Commands.sequence(
                Commands.runOnce(() -> heldPiece = PieceType.NONE),
                new ParallelCommandGroup(
                        elevator.L1(),
                        arm.moveCommand(ArmConstants.CORAL_HANDOFF_PREP_ANGLE)
                ),
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(arm::isHoldingItem).andThen(new WaitCommand(0.4)).withTimeout(ArmConstants.INTAKE_TIMEOUT.in(Seconds)),
                        new ParallelCommandGroup(
                                elevator.getMoveToPositionCommand(ElevatorConstants.HOME_POSITION),
                                Commands.runOnce(arm::startIntake)
                        )
                ),
                new InstantCommand(() -> SmartDashboard.putBoolean("AUTO/PICKUP NOW STOW", true)),
                stow(),
                Commands.runOnce(() -> {
                    if (arm.isHoldingItem()) {
                        arm.holdItem();
                        heldPiece = PieceType.CORAL;
                        coralInCradle = indexer.hasCoral();
                        currentLevel = Level.L1;
                    }
                    arm.stopRollers();
                })
        ).withName("CoralCradlePickup")
                .finallyDo(() -> heldPiece = PieceType.CORAL);
    }

    public Command manualCoralCradlePickup() {
        return Commands.sequence(
                        Commands.runOnce(() -> heldPiece = PieceType.NONE),
                        new ParallelCommandGroup(
                                elevator.L1(),
                                arm.moveCommand(ArmConstants.CORAL_HANDOFF_PREP_ANGLE)
                        ),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(arm::isHoldingItem).andThen(new WaitCommand(0.4)).withTimeout(ArmConstants.INTAKE_TIMEOUT.in(Seconds)),
                                new ParallelCommandGroup(
                                        elevator.getMoveToPositionCommand(ElevatorConstants.HOME_POSITION),
                                        Commands.runOnce(arm::startIntake)
                                )
                        ),
                        new InstantCommand(() -> SmartDashboard.putBoolean("AUTO/PICKUP NOW STOW", true)),
                        Commands.runOnce(() -> {
                            if (arm.isHoldingItem()) {
                                arm.holdItem();
                                heldPiece = PieceType.CORAL;
                                coralInCradle = indexer.hasCoral();
                                currentLevel = Level.L1;
                            }
                            arm.stopRollers();
                        })
                ).withName("CoralCradlePickup")
                .finallyDo(() -> heldPiece = PieceType.CORAL);
    }

    public Command algaeGroundPickup() {
        if(arm.isHoldingItem()) return Commands.none();
        return Commands.sequence(
                Commands.runOnce(() -> {
                    heldPiece = PieceType.NONE;
                    arm.startIntake();
                }),
                new ParallelDeadlineGroup(
                        Commands.waitUntil(arm::isHoldingItem).andThen(new WaitCommand(0.4)),
                        new ParallelCommandGroup(
                                elevator.AL1(),
                                arm.moveCommand(ArmConstants.ALGAE_GROUND_INTAKE_ANGLE)
                        )
                ),
                Commands.runOnce(() -> {
                    if (arm.isHoldingItem()) {
                        arm.holdItem();
                        heldPiece = PieceType.ALGAE;
                        coralInCradle = indexer.hasCoral();
                        currentLevel = Level.L1;
                    } else {
                        arm.stopRollers();
                        heldPiece = PieceType.NONE;
                        coralInCradle = indexer.hasCoral();
                    }
                }),
                Commands.either(
                        new ParallelCommandGroup(
                                elevator.L2(),
                                arm.moveCommand(ArmConstants.ALGAE_HOLD_ANGLE)
                        ),
                        Commands.none(),
                        () -> heldPiece == PieceType.ALGAE
                )
        ).withName("AlgaeGroundPickup")
                .finallyDo(interrupted -> {
                    if (!arm.isHoldingItem()) {
                        arm.stopRollers();
                        heldPiece = PieceType.NONE;
                    }
                });
    }

    public Command algaeReefPickup(Level level) {
        if (level != Level.L2 && level != Level.L3 || arm.isHoldingItem()) {
            return Commands.none();
        }
        return Commands.sequence(
                Commands.runOnce(() -> {
                    heldPiece = PieceType.NONE;
                    arm.startIntake();
                }),
                new ParallelDeadlineGroup(
                        Commands.waitUntil(arm::isHoldingItem).andThen(new WaitCommand(0.4)),
                        new ParallelCommandGroup(
                                new RunCommand(arm::startIntake),
                                elevator.getAlgaeLevelCommand(level.elevatorLevel()),
                                arm.moveCommand(ArmConstants.ALGAE_REEF_INTAKE_ANGLE)
                        )
                ),
                Commands.runOnce(() -> {
                    if (arm.isHoldingItem()) {
                        arm.holdItem();
                        heldPiece = PieceType.ALGAE;
                        coralInCradle = indexer.hasCoral();
                        currentLevel = level;
                    } else {
                        arm.stopRollers();
                        heldPiece = PieceType.NONE;
                        coralInCradle = indexer.hasCoral();
                    }
                }),
                Commands.either(
                        new ParallelCommandGroup(
                                elevator.stow(),
                                arm.moveCommand(ArmConstants.ALGAE_HOLD_ANGLE)
                        ),
                        Commands.none(),
                        () -> heldPiece == PieceType.ALGAE
                )
        ).withName("AlgaeReefPickup" + level.name())
                .finallyDo(interrupted -> {
                    if (!arm.isHoldingItem()) {
                        arm.stopRollers();
                        heldPiece = PieceType.NONE;
                    }
                });
    }

    public Command prepareLevel(Level level) {
        return Commands.defer(() -> {
            switch (heldPiece) {
                case CORAL:
                    return coralPrepare(level);
                case ALGAE:
                    return algaePrepare(level);
                case NONE:
                default:
                    if (coralInCradle || autoPickupCommand != null) {
                        return Commands.waitUntil(() -> heldPiece == PieceType.CORAL)
                                .andThen(coralPrepare(level));
                    }
                    if (level == Level.L1) {
                        return algaeGroundPickup();
                    } else if (level == Level.L2 || level == Level.L3) {
                        return algaeReefPickup(level);
                    }
                    return Commands.none();
            }
        }, Set.of(this));
    }

    public Command scoreHeldPiece() {
        return Commands.defer(() -> switch (heldPiece) {
            case CORAL -> coralScore(currentLevel);
            case ALGAE -> algaeScore(currentLevel);
            default -> Commands.none();
        }, Set.of(this)).finallyDo(() -> {
            heldPiece = PieceType.NONE;
            coralInCradle = indexer.hasCoral();
            arm.stopRollers();
        });
    }

    public Command stow() {
        return Commands.sequence(
                elevator.stow(),
                arm.stowCommand()
        ).withName("SuperstructureStow");
    }

    public Command coralPrepare(Level level) {
        Command armCommand = switch (level) {
            case L4 -> arm.moveCommand(ArmConstants.CORAL_L4_BEFORE_SCORING);
            case L3 -> arm.moveCommand(ArmConstants.CORAL_L3_BEFORE_SCORING);
            case L2 -> arm.moveCommand(ArmConstants.CORAL_L2_BEFORE_SCORING);
            case L1 -> arm.moveCommand(ArmConstants.CORAL_L1_BEFORE_SCORING);
        };

        return new ParallelCommandGroup(
                elevator.getLevelCommand(level.elevatorLevel()),
                armCommand,
                Commands.runOnce(() -> currentLevel = level)
        );
    }

    public Command coralScore(Level level) {
        Command armCommand = switch (level) {
            case L4 -> arm.moveCommand(ArmConstants.CORAL_L4_SCORING);
            case L3 -> arm.moveCommand(ArmConstants.CORAL_L3_SCORING);
            case L2 -> arm.moveCommand(ArmConstants.CORAL_L2_SCORING);
            case L1 -> arm.moveCommand(ArmConstants.CORAL_L1_SCORING);
        };

        Angle releaseAngle = switch (level) {
            case L4 -> ArmConstants.CORAL_L4_RELEASE;
            case L3 -> ArmConstants.CORAL_L3_RELEASE;
            case L2 -> ArmConstants.CORAL_L2_RELEASE;
            case L1 -> ArmConstants.CORAL_L1_SCORING;
        };

        return Commands.sequence(
                new ParallelCommandGroup(
                        armCommand,
                        Commands.sequence(
                                new WaitUntilCommand(() -> arm.getAngle().isNear(releaseAngle, Degrees.of(5)))
                                        .withTimeout(.5),
                                new ParallelCommandGroup(
                                        Commands.either(new RunCommand(arm::startOutputSlow), new RunCommand(arm::startOutput), () -> level == Level.L1),
                                        this.run(() -> drivetrain.setChassisSpeeds(new ChassisSpeeds(-1.5, 0.0, 0.0)))
                                ).withTimeout(.5).finallyDo(
                                        () -> {
                                            drivetrain.setChassisSpeeds(new ChassisSpeeds());
                                            heldPiece = PieceType.NONE;
                                            coralInCradle = indexer.hasCoral();
                                            arm.stopRollers();

                                        }
                                )
                        )
                ),
                        new InstantCommand(() -> SmartDashboard.putBoolean("AUTO/FINISH AUTOSCORE", true)),
                stow()
        )
                .withName("CoralScore" + level.name());
    }

    private Command algaePrepare(Level level) {
        Command armCommand = switch (level) {
            case L4 -> arm.algaeBargeBeforeScoringCommand();
            case L3, L2 -> arm.moveCommand(ArmConstants.ALGAE_REEF_INTAKE_ANGLE);
            case L1 -> arm.moveCommand(ArmConstants.ALGAE_PROCESSOR_BEFORE_SCORING);
        };

        Command elevatorCommand = switch (level) {
            case L4 -> elevator.AL4();
            case L3 -> elevator.AL3();
            case L2 -> elevator.AL2();
            case L1 -> elevator.AL1();
        };

        return new ParallelCommandGroup(
                elevatorCommand,
                armCommand
        ).andThen(Commands.runOnce(() -> currentLevel = level));
    }

    private Command algaeScore(Level level) {
        return switch (level) {
            case L4 -> Commands.sequence(
                    elevator.AL4(),
                    arm.algaeBargeScoringCommand(),
                    finalizeAlgaeScore()
            ).withName("AlgaeScoreBarge");
            case L1 -> Commands.sequence(
                    elevator.AL1(),
                    arm.algaeProcessorScoringCommand(),
                    finalizeAlgaeScore()
            ).withName("AlgaeScoreProcessor");
            case L3, L2 -> Commands.none();
        };
    }

    private Command finalizeAlgaeScore() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    if (!hasGamePiece()) {
                        heldPiece = PieceType.NONE;
                    }
                    coralInCradle = indexer.hasCoral();
                    arm.stopRollers();
                })
        );
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

    /**
     * Debug helper that bypasses the superstructure state machine and directly moves the arm.
     */
    public Command debugArmToAngle(Angle angle) {
        return arm.moveCommand(angle).withName("DebugArmTo" + angle);
    }

    /**
     * Debug helper that bypasses the superstructure state machine and directly moves the elevator.
     */
    public Command debugElevatorToHeight(Distance height) {
        return elevator.getMoveToPositionCommand(height).withName("DebugElevatorTo" + height);
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

    public Command reverseCommand(){
        return indexer.reverseCommand().alongWith(intake.reverseRollersCommand());
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
        return arm.runOuttakeCommand().alongWith(new InstantCommand(() -> heldPiece = PieceType.NONE));
    }

    public Command armStop() {
        return arm.stopRollersCommand();
    }
}
