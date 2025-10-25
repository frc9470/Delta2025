package com.team9470.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team9470.subsystems.Swerve;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.elevator.Elevator;
import com.team9470.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import static edu.wpi.first.units.Units.Degrees;

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

    public Command scoreL4(){
        return elevator.L4().andThen(
                superstructure.score().withTimeout(SCORING_DELAY))
                .andThen(elevator.L0());
    }

    public Command scoreL4WaitLower(Command driveAway, double delay){
        return elevator.L4()
                .andThen(
                    superstructure.score().withTimeout(SCORING_DELAY)
                )
                .andThen(
                        driveAway
                        .alongWith(
                                new WaitCommand(delay).andThen(elevator.L0())
                        )
                );

    }

    public Command scoreL4AutoWaitLower(Command driveAway, double delay, int branchID){
        return AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(branchID, Superstructure.Level.L4), swerve).andThen(driveAway
                .alongWith(
                        new WaitCommand(delay).andThen(elevator.L0())
                ));
    }

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
                return AllianceFlipUtil.apply(new Pose2d(1.8302104473114014, 6.76573371887207, Rotation2d.fromDegrees(-54)));
        else if(side==LollipopSide.MIDDLE)
                return AllianceFlipUtil.apply(new Pose2d(1.8302104473114014, 4.0085768699646, Rotation2d.fromDegrees(-54)));
        return AllianceFlipUtil.apply(new Pose2d(1.8302104473114014, 2.1849470138549805, Rotation2d.fromDegrees(-54)));
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
                .andThen(superstructure.waitForIntake().deadlineFor(new DriveToPose(() -> getSourcePose(top), swerve, true, -1)));

    }

    public Command alignToLollipop(LollipopSide side){
        return new DriveToPose(() -> getLollipopPose(side), swerve, true, 3).raceWith(superstructure.waitForIntake());
    }

    public Command alignToLollipopWait(LollipopSide side){
        return new DriveToPose(() -> getLollipopPose(side), swerve, true, 10)
                .andThen(superstructure.waitForIntake().deadlineFor(new DriveToPose(() -> getLollipopPose(side), swerve, true, -1)));
    }

    public Command scoreCoral() {
        return superstructure.score();
    }

    public AutoRoutine getOneCoralMiddleAutoNormal(){
        AutoRoutine routine = autoFactory.newRoutine("1CMN");

        AutoTrajectory startToC3 = routine.trajectory("S-3");
        routine.active().onTrue(
                startToC3.resetOdometry().andThen(
                        scoreL4AutoWaitLower(new InstantCommand(), SCORING_DELAY, 7)
                )

        );
        return routine;
    }

    public AutoRoutine getOneCoralMiddleAutoChoreo(){
        AutoRoutine routine = autoFactory.newRoutine("1CMC");
        AutoTrajectory startToC3 = routine.trajectory("S-3");

        routine.active().onTrue(
                startToC3.resetOdometry()
                        .andThen(startToC3.cmd())
        );

        startToC3.done().onTrue(
                scoreL4WaitLower(new InstantCommand(), SCORING_DELAY)
        );
        return routine;
    }

    public AutoRoutine getOneCoralMiddleAutoAlign(){
        AutoRoutine routine = autoFactory.newRoutine("1CMA");

        routine.active().onTrue(
                Commands.sequence(
                        new WaitUntilCommand(superstructure::hasCoralInArm),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(7, Superstructure.Level.L4), swerve)
                )

        );

        return routine;
    }
    private static final Pose2d START_TOP = new Pose2d(new Translation2d(7, 5), new Rotation2d(Degrees.of(240)));
    private static final Pose2d START_BOTTOM = new Pose2d(new Translation2d(7, 3), new Rotation2d(Degrees.of(120)));

    public AutoRoutine getFiveCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CTA");
        // TODO: define odom start position and reset odometry to that position, analogous to "startToC5.resetOdometry()" choreo call

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(9, Superstructure.Level.L4), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L3), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSourceWait(true)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L3), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("5CBA");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(4, Superstructure.Level.L4), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L3), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSourceWait(false)),

                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L3), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralTopAutoAlignNoWait() {
        AutoRoutine routine = autoFactory.newRoutine("5CTA-NW");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(9, Superstructure.Level.L4), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L4), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L2), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSource(true)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L2), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFiveCoralBottomAutoAlignNoWait() {
        AutoRoutine routine = autoFactory.newRoutine("5CBA-NW");

        routine.active().onTrue(
                Commands.sequence(
                        AutoScoring.autoScoreStraight(superstructure, new AutoScoring.CoralObjective(4, Superstructure.Level.L4), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L4), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L3), swerve),
                        Commands.parallel(elevator.L0())
                                .withDeadline(alignToSource(false)),
                        new WaitCommand(INTAKE_DELAY),
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L3), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFourCoralTopAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("4CTA");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(9, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(true)),

                        // Second score - from source
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToLollipopWait(LollipopSide.MIDDLE)),

                        // Third score - from middle lollipop
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(true)),

                        // Fourth score - from source again
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(12, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFourCoralBottomAutoAlign() {
        AutoRoutine routine = autoFactory.newRoutine("4CBA");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(4, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(false)),

                        // Second score - from source
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToLollipopWait(LollipopSide.MIDDLE)),

                        // Third score - from middle lollipop
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToSourceWait(false)),

                        // Fourth score - from source again
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(1, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFourCoralTopAutoSweep() {
        AutoRoutine routine = autoFactory.newRoutine("4CTS");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        Commands.sequence(
                                new WaitUntilCommand(superstructure::hasCoralInArm),
                                AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(9, Superstructure.Level.L4), swerve)
                        ),
                        superstructure.stow()
                                .withDeadline(getSweepMotionCommand(true)),

                        // Second score - from sweep intake

                        Commands.sequence(
                                new WaitUntilCommand(superstructure::hasCoralInArm),
                            AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(10, Superstructure.Level.L4), swerve)
                        ),
                        superstructure.stow()
                                .withDeadline(alignToLollipopWait(LollipopSide.MIDDLE)),

                        // Third score - from middle lollipop
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(11, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(getSweepMotionCommand(true)),

                        // Fourth score - from sweep intake again
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(12, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }

    public AutoRoutine getFourCoralBottomAutoSweep() {
        AutoRoutine routine = autoFactory.newRoutine("4CBS");

        routine.active().onTrue(
                Commands.sequence(
                        // First score - start position
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(4, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(getSweepMotionCommand(false)),

                        // Second score - from sweep intake
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(3, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(alignToLollipopWait(LollipopSide.MIDDLE)),

                        // Third score - from middle lollipop
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(2, Superstructure.Level.L4), swerve),
                        superstructure.stow()
                                .withDeadline(getSweepMotionCommand(false)),

                        // Fourth score - from sweep intake again
                        AutoScoring.autoScoreWithTimeout(superstructure, new AutoScoring.CoralObjective(1, Superstructure.Level.L4), swerve)
                )
        );

        return routine;
    }


    // basic leave auto
    public Command getBasicAutoCommand() {
        ChassisSpeeds initial_speed = new ChassisSpeeds(1, 0, 0);
        ChassisSpeeds final_speed = new ChassisSpeeds(0, 0, 0);

        return this.runEnd(
            () -> swerve.setChassisSpeeds(initial_speed),
            () -> swerve.setChassisSpeeds(final_speed)
        );
    }

}
