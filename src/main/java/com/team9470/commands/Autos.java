package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import com.team9470.subsystems.Swerve;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.elevator.Elevator;
import com.team9470.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;

public class Autos extends SubsystemBase{
    private final AutoFactory autoFactory;
    private final Elevator elevator;
    private final Swerve swerve;
    private final Superstructure superstructure;

    public Autos(Superstructure superstructure, Swerve swerve) {
        this.autoFactory = swerve.createAutoFactory((sample, isStart) -> {
        });
        this.elevator = superstructure.getElevator();
        this.swerve = swerve;
        this.superstructure = superstructure;
    }

    public static final double SCORING_DELAY = 0.3;
    public static final double INTAKE_DELAY = 0.3;
    private static final double ELEVATOR_DELAY = 0.7;

    public Pose2d getSourcePose(boolean top){ // REPLACE POSES IN TESTING
        return AllianceFlipUtil.apply(
                top ? new Pose2d(1.8302104473114014, 6.76573371887207, Rotation2d.fromDegrees(-54)) : new Pose2d(1.966275930404663, 1.436502456665039, Rotation2d.fromDegrees(54))
        );
    }

    public enum LollipopSide{
        TOP,
        MIDDLE,
        BOTTOM
    }

    public Pose2d getLollipopPose(LollipopSide side){ // REPLACE POSES IN TESTING
        if(side==LollipopSide.TOP)
                return AllianceFlipUtil.apply(new Pose2d(1.8302104473114014, 6.76573371887207, Rotation2d.fromDegrees(0)));
        else if(side==LollipopSide.MIDDLE)
                return AllianceFlipUtil.apply(new Pose2d(1.8302104473114014, 4.0085768699646, Rotation2d.fromDegrees(0)));
        return AllianceFlipUtil.apply(new Pose2d(1.8302104473114014, 2.1849470138549805, Rotation2d.fromDegrees(0)));
    }

    /**
     * Creates a command that performs a sweeping motion through the specified waypoints.
     * The robot will move through each position in sequence, maintaining the specified heading at each waypoint.
     *
     * @return Command that executes the sweeping motion
     */
    public Command getSweepMotionCommand(boolean top) {
        return getSweepMotionCommand(top, false);
    }

    public Command getSweepMotionCommand(boolean top, boolean useDriveToPose) {
        // Define different waypoints for top vs bottom source
        Pose2d[] waypoints;

        if (top) {
            waypoints = new Pose2d[]{
                    new Pose2d(2.8092904090881348, 7.4921770095825195, Rotation2d.fromDegrees(0)),
                    new Pose2d(2.0143749713897705, 7.4921770095825195, Rotation2d.fromDegrees(19.5)),
                    new Pose2d(1.2194594144821167, 7.164858818054199, Rotation2d.fromDegrees(36)),
                    new Pose2d(0.728482186794281, 6.627121925354004, Rotation2d.fromDegrees(55)),
                    new Pose2d(0.5648230910301208, 5.972485542297363, Rotation2d.fromDegrees(90))
            };
        } else {
            waypoints = new Pose2d[]{
                    new Pose2d(2.8092904090881348, 0.7078229904174805, Rotation2d.fromDegrees(0)),
                    new Pose2d(2.0143749713897705, 0.7078229904174805, Rotation2d.fromDegrees(-19.5)),
                    new Pose2d(1.2194594144821167, 1.0351411819458008, Rotation2d.fromDegrees(-36)),
                    new Pose2d(0.728482186794281, 1.572878074645996, Rotation2d.fromDegrees(-55)),
                    new Pose2d(0.5648230910301208, 2.227514457702637, Rotation2d.fromDegrees(-90))
            };
        }

        // Apply alliance flipping to all waypoints
        for (int i = 0; i < waypoints.length; i++) {
            waypoints[i] = AllianceFlipUtil.apply(waypoints[i]);
        }

        if (useDriveToPose) {
            Command[] driveCommands = new Command[waypoints.length];
            for (int i = 0; i < waypoints.length; i++) {
                final int index = i; // Capture for lambda
                driveCommands[i] = new DriveToPose(() -> waypoints[index], swerve, true, 0.5);
            }
            return Commands.sequence(driveCommands);
        }

        return new DriveThroughWaypoints(swerve, waypoints, true, 0.5);
    }


    public Command alignToSource(boolean top) {

        return new DriveToPose(() -> getSourcePose(top), swerve, true, 3).raceWith(superstructure.waitForIntake());
    }

    public Command alignToSourceWait(boolean top){
        return new DriveToPose(() -> getSourcePose(top), swerve, true, 10)
                .andThen(superstructure.waitForIntake().deadlineFor(new DriveToPose(() -> getSourcePose(top), swerve, true, -1)))
                .alongWith(superstructure.intakeCoralGround()).andThen(superstructure.stopIntakeGround());

    }

    public Command alignToLollipop(LollipopSide side){
        return new DriveToPose(() -> getLollipopPose(side), swerve, true, 3).raceWith(superstructure.waitForIntake());
    }

    public Command alignToLollipopWait(LollipopSide side){
        return new DriveToPose(() -> getLollipopPose(side), swerve, true, 10)
                .andThen(superstructure.waitForIntake().deadlineFor(new DriveToPose(() -> getLollipopPose(side), swerve, true, -1)))
                .alongWith(superstructure.intakeCoralGround()).andThen(superstructure.stopIntakeGround());
    }


    public AutoRoutine getOneCoralMiddleAutoAlign(){
        AutoRoutine routine = autoFactory.newRoutine("1CMA");

        routine.active().onTrue(
                Commands.sequence(
                        new WaitUntilCommand(superstructure::hasCoralInArm),
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(7, Superstructure.Level.L4), swerve)
                )

        );

        return routine;
    }

    public AutoRoutine getFourCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("4CTA");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(9, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(true)),

                        // Second score - from source
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToLollipopWait(LollipopSide.MIDDLE)),

                        // Third score - from middle lollipop
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(true)),

                        // Fourth score - from source again
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(12, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFourCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("4CBA");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(4, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(false)),

                        // Second score - from source
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToLollipopWait(LollipopSide.MIDDLE)),

                        // Third score - from middle lollipop
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(false)),

                        // Fourth score - from source again
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(1, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }

    public Command intakeSweep(boolean top){
        return superstructure.stow().withTimeout(0.02)
                .andThen(
                        superstructure.waitForIntake()
                                .deadlineFor(getSweepMotionCommand(top))
                                .alongWith(superstructure.intakeCoralGround().andThen(superstructure.coralCradlePickup()))

                );
    }

    public AutoRoutine getFourCoralTopAutoSweep() {
        AutoRoutine routine = autoFactory.newRoutine("4CTS");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(9, Superstructure.Level.L4), swerve),
                        intakeSweep(true),

                        // Second score - from sweep intake
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L4), swerve),
                        intakeSweep(true),

                        // Third score - from middle lollipop
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L4), swerve),
                        intakeSweep(true),

                        // Fourth score - from sweep intake again
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(12, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFourCoralBottomAutoSweep() {
        AutoRoutine routine = autoFactory.newRoutine("4CBS");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(4, Superstructure.Level.L4), swerve),
                        intakeSweep(false),

                        // Second score - from sweep intake
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L4), swerve),
                        intakeSweep(false),

                        // Third score - from middle lollipop
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L4), swerve),
                        intakeSweep(false),

                        // Fourth score - from sweep intake again
                        AutoScoring.autoScore(superstructure, new AutoScoring.CoralObjective(1, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }


    // basic leave auto as routine
    public AutoRoutine getBasicLeaveAutoRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("ONLYLEAVE");

        routine.active().onTrue(
            Commands.runEnd(
                () -> swerve.setChassisSpeeds(new ChassisSpeeds(1, 0, 0)),
                () -> swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)),
                swerve
            ).withTimeout(0.6) // move for a short duration, then stop
        );

        return routine;
    }

}
