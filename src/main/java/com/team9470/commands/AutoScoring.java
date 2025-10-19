package com.team9470.commands;

import com.team9470.Constants;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Set;

import static edu.wpi.first.units.Units.Meters;

public class AutoScoring {

    public enum ReefPosition {
        LEFT, RIGHT
    }

    private CoralObjective coralObjective = AutoScoring.CoralObjective.NONE;
    private final Swerve drivetrain;

    public AutoScoring(Swerve drivetrain){
        this.drivetrain = drivetrain;
    }

    public Command autoScore(Superstructure superstructure) {
        return new DeferredCommand(() -> {
            // Automatically find the closest reef position
            CoralObjective closestObjective = findClosestReefPosition(coralObjective.level());
            
            Command driveToScore = new DriveToPose(closestObjective::getScoringPose, drivetrain)
                    .alongWith(
                            new WaitUntilCommand(() -> closeEnough(closestObjective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                    .andThen(superstructure.waitForIntake().asProxy())
                                    .andThen(superstructure.raise(closestObjective.level()).asProxy())
                    );
            System.out.println("Auto-scoring with closest coralobjective: " + closestObjective);
            return driveToScore;
        }, Set.of(drivetrain)).asProxy().andThen(superstructure.score().asProxy());
    }

    public static Command autoScore(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                .andThen(superstructure.waitForIntake().asProxy())
                                .andThen(superstructure.raise(objective.level))
                );
        return driveToScore.andThen(superstructure.score().asProxy());
    }

    public static Command autoScoreWithTimeout(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE))
                                .andThen(superstructure.raise(objective::level))
                );
        return driveToScore.andThen(superstructure.score().withTimeout(1).until(() -> !superstructure.hasGamePiece()).asProxy());
    }

    public static Command autoScoreStraight(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
        // First drive to the scoring position while raising the superstructure.
        Command driveToScore = new DriveToPose(objective::getScoringPose, drivetrain, true)
                .alongWith(
                        new WaitUntilCommand(() -> closeEnough(objective, Constants.DriverAssistConstants.RAISE_DISTANCE.times(2)))
                                .andThen(superstructure.raise(objective::level))
                );
        return driveToScore.andThen(superstructure.score().withTimeout(1).until(() -> !superstructure.hasGamePiece()).asProxy());
    }

    // Auto-score to the closest reef position with specified level
    public static Command autoScoreClosest(Superstructure superstructure, int level, Swerve drivetrain) {
        return autoScore(superstructure, findClosestReefPosition(level), drivetrain);
    }

    // Auto-score with timeout to the closest reef position
    public static Command autoScoreWithTimeoutClosest(Superstructure superstructure, int level, Swerve drivetrain) {
        return autoScoreWithTimeout(superstructure, findClosestReefPosition(level), drivetrain);
    }

    // Auto-score straight to the closest reef position
    public static Command autoScoreStraightClosest(Superstructure superstructure, int level, Swerve drivetrain) {
        return autoScoreStraight(superstructure, findClosestReefPosition(level), drivetrain);
    }

    public Command autoScoreNoDrive(Superstructure superstructure) {
        return new DeferredCommand(() -> {
            return superstructure.raise(coralObjective.level).andThen(superstructure.score());
        }, Set.of());
    }

    private Pose2d computeDriveBackPose(Pose2d scoringPose) {
        // Drive backward 1 meter relative to the scoring pose.
        Translation2d backOffset = new Translation2d(-.5, 0.0).rotateBy(scoringPose.getRotation());
        return new Pose2d(scoringPose.getTranslation().plus(backOffset), scoringPose.getRotation());
    }

    public Command driveBack(){
        Pose2d scoringPose = coralObjective.getScoringPose();
        Pose2d driveBackPose = computeDriveBackPose(scoringPose);

        return new DriveToPose(() -> driveBackPose, drivetrain);

    }

    private static boolean closeEnough(CoralObjective objective, Distance distance){
        Pose2d scoringPose = objective.getScoringPose();
        Pose2d currentPose = Swerve.getInstance().getPose();
        return currentPose.getTranslation().getDistance(scoringPose.getTranslation()) < distance.in(Meters);
    }

    // Helper method to find the closest reef position automatically
    private static CoralObjective findClosestReefPosition(int level) {
        Pose2d[] reefPoses = Constants.DriverAssistConstants.getReefPositions();
        Pose2d currentPose = Swerve.getInstance().getPose();

        // Find the closest reef face by checking distances to both positions on each face.
        double shortestDistance = Double.MAX_VALUE;
        int closestFace = -1;
        ReefPosition closestPosition = ReefPosition.LEFT;
        
        for (int face = 0; face < 6; face++) {
            // Check left position (even indices: 0, 2, 4, 6, 8, 10)
            Pose2d leftPose = reefPoses[face * 2];
            double leftDistance = currentPose.getTranslation().getDistance(leftPose.getTranslation());
            
            // Check right position (odd indices: 1, 3, 5, 7, 9, 11)
            Pose2d rightPose = reefPoses[face * 2 + 1];
            double rightDistance = currentPose.getTranslation().getDistance(rightPose.getTranslation());
            
            // Find the closer position on this face
            if (leftDistance < rightDistance && leftDistance < shortestDistance) {
                shortestDistance = leftDistance;
                closestFace = face;
                closestPosition = ReefPosition.LEFT;
            } else if (rightDistance < shortestDistance) {
                shortestDistance = rightDistance;
                closestFace = face;
                closestPosition = ReefPosition.RIGHT;
            }
        }
        
        CoralObjective closestObjective = new CoralObjective(closestFace, closestPosition, level);
        System.out.println("Auto-selected closest reef - Face: " + closestFace + ", Position: " + closestPosition + ", Level: " + level);
        SmartDashboard.putNumber("AutoScoring/Auto Face", closestFace);
        SmartDashboard.putString("AutoScoring/Auto Position", closestPosition.toString());
        
        return closestObjective;
    }

    // Update the reef position (left/right) used for automatic scoring.
    public void setPosition(ReefPosition position) {
        coralObjective = coralObjective.updatePosition(position);
        System.out.println("Reef Position: " + position);
        SmartDashboard.putString("AutoScoring/Position", position.toString());
    }


    public void setLevel(int level) {
        coralObjective = coralObjective.updateLevel(level);
        SmartDashboard.putString("AutoScoring/Level: ", "L" + level);
    }

    public void updateClosestReefPos() {
        // Use the helper method to find closest position and update the current objective
        coralObjective = findClosestReefPosition(coralObjective.level());
        System.out.println("Updated coralObjective to closest reef position: " + coralObjective);
    }

    // Starting facing the driver station, moving counterclockwise
    public record CoralObjective(int face, ReefPosition position, int level){
        public static final CoralObjective NONE = new CoralObjective(0, ReefPosition.LEFT, 0);

        public Pose2d getScoringPose(){
            if (level == 1) return Constants.DriverAssistConstants.getL1Pose(this);
            
            // Convert face + position to branchId for reef positions array
            int branchId = face * 2 + (position == ReefPosition.LEFT ? 0 : 1);
            return Constants.DriverAssistConstants.getReefPositions()[branchId];
        }

        public int getFace(){
            return face;
        }

        public int getBranchId() {
            // Convert face + position back to branchId for backward compatibility
            return face * 2 + (position == ReefPosition.LEFT ? 0 : 1);
        }

        public int getAlgaeLevel(){
            // starting with (0, 1) = 3, then (2, 3) = 2, each reef alternates level between 2 and 3
            return face % 2 == 0 ? 3 : 2;
        }

        public CoralObjective updateFaceAndPosition(int face, ReefPosition position){
            return new CoralObjective(face, position, level);
        }

        public CoralObjective updatePosition(ReefPosition position){
            return new CoralObjective(face, position, level);
        }

        public CoralObjective updateLevel(int level){
            return new CoralObjective(face, position, level);
        }

        // Legacy method for backward compatibility
        @Deprecated
        public CoralObjective updateBranchId(int branchId){
            int newFace = branchId / 2;
            ReefPosition newPosition = (branchId % 2 == 0) ? ReefPosition.LEFT : ReefPosition.RIGHT;
            return new CoralObjective(newFace, newPosition, level);
        }
    }
}
