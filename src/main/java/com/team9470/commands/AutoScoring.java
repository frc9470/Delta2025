package com.team9470.commands;

import com.team9470.constants.DriverAssistConstants;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.Superstructure.Level;
import com.team9470.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.Set;

public class AutoScoring {

    private CoralObjective coralObjective = AutoScoring.CoralObjective.NONE;
    private final Swerve drivetrain;

    public AutoScoring(Swerve drivetrain){
        this.drivetrain = drivetrain;
    }


    public static Command autoScore(Superstructure superstructure, CoralObjective objective, Swerve drivetrain) {
//        return new WaitUntilCommand(superstructure::hasCoralInArm).andThen(
                return createAutoScoreCommand(superstructure, objective, drivetrain, false);
                //);
    }

    public Command autoAlign(Superstructure superstructure) {
        return Commands.defer(() -> {
            CoralObjective objective = new CoralObjective(findClosestBranch(), superstructure.currentLevel);
            coralObjective = objective;
            SmartDashboard.putNumber("AutoScoring/Branch ID", objective.branchId());
            SmartDashboard.putString("AutoScoring/Level", objective.level().name());
            return createAutoAlignCommand(objective, drivetrain, false);
        }, Set.of(drivetrain));
    }

    public Command autoAlign(Superstructure superstructure, Side side) {
        return Commands.defer(() -> {
            int branchId = findClosestBranch(side);
            CoralObjective objective = new CoralObjective(branchId, superstructure.currentLevel);
            coralObjective = objective;
            SmartDashboard.putNumber("AutoScoring/Branch ID", branchId);
            SmartDashboard.putString("AutoScoring/Level", objective.level().name());
            return createAutoAlignCommand(objective, drivetrain, false);
        }, Set.of(drivetrain));
    }

    public void updateClosestReefPos() {
        setBranch(findClosestBranch());
    }

    private int findClosestBranch() {
        Pose2d[] reefPoses = DriverAssistConstants.getReefPositions();
        Pose2d currentPose = drivetrain.getPose();

        double shortestDistance = Double.MAX_VALUE;
        int closestPoseId = coralObjective.branchId();
        for (int i = 0; i < reefPoses.length; i++) {
            Pose2d pose = reefPoses[i];
            double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestPoseId = i;
            }
        }
        return closestPoseId;
    }

    private int findClosestBranch(Side side) {
        Pose2d[] reefPoses = DriverAssistConstants.getReefPositions();
        Pose2d currentPose = drivetrain.getPose();

        int closestFace = -1;
        double shortestDistance = Double.MAX_VALUE;

        for (int face = 0; face < reefPoses.length / 2; face++) {
            Translation2d faceCenter = getFaceCenter(reefPoses, face);
            double distance = currentPose.getTranslation().getDistance(faceCenter);
            if (distance < shortestDistance) {
                shortestDistance = distance;
                closestFace = face;
            }
        }

        if (closestFace < 0) {
            return findClosestBranch();
        }

        int firstBranch = closestFace * 2;
        int secondBranch = firstBranch + 1;

        Translation2d faceCenter = getFaceCenter(reefPoses, closestFace);
        Translation2d forward = new Translation2d(
                reefPoses[firstBranch].getRotation().getCos(),
                reefPoses[firstBranch].getRotation().getSin());

        Translation2d firstOffset = reefPoses[firstBranch].getTranslation().minus(faceCenter);

        double firstCross = forward.getX() * firstOffset.getY() - forward.getY() * firstOffset.getX();
        boolean firstIsLeft = firstCross >= 0.0;

        if (side == Side.LEFT) {
            return firstIsLeft ? firstBranch : secondBranch;
        }

        return firstIsLeft ? secondBranch : firstBranch;
    }

    private Translation2d getFaceCenter(Pose2d[] reefPoses, int face) {
        Translation2d first = reefPoses[face * 2].getTranslation();
        Translation2d second = reefPoses[face * 2 + 1].getTranslation();
        return new Translation2d((first.getX() + second.getX()) / 2.0, (first.getY() + second.getY()) / 2.0);
    }

    // Update the reef value used for automatic level selection.
    public void setBranch(int branchId) {
        coralObjective = coralObjective.updateBranchId(branchId);
        System.out.println("Branch ID: " + branchId);
        SmartDashboard.putNumber("AutoScoring/Branch ID", branchId);
    }

    public void setLevel(int level) {
        coralObjective = coralObjective.updateLevel(fromInt(level));
        SmartDashboard.putString("AutoScoring/Level", coralObjective.level().name());
    }

    public void setLevel(Level level) {
        coralObjective = coralObjective.updateLevel(level);
        SmartDashboard.putString("AutoScoring/Level", level.name());
    }

    private static Level fromInt(int level) {
        return switch (level) {
            case 4 -> Level.L4;
            case 3 -> Level.L3;
            case 2 -> Level.L2;
            default -> Level.L1;
        };
    }

    private static Command createAutoScoreCommand(Superstructure superstructure, CoralObjective objective, Swerve drivetrain, boolean straight) {
        Command drive = straight
                ? new DriveToPose(objective::getScoringPose, drivetrain, true)
                : new DriveToPose(objective::getScoringPose, drivetrain);

        Command prepare = superstructure.coralPrepare(objective.level());

        Command driveAndPrepare = Commands.parallel(drive, new WaitUntilCommand(superstructure::hasCoralInArm).andThen(prepare));

        return driveAndPrepare.andThen(new InstantCommand(() -> SmartDashboard.putBoolean("AUTO/DRIVE FINISHED", true)))
                .andThen(superstructure.coralScore(objective.level()).withTimeout(1.5));
    }

    private static Command createAutoAlignCommand(CoralObjective objective, Swerve drivetrain, boolean straight) {
        return straight
                ? new DriveToPose(objective::getScoringPose, drivetrain, true)
                : new DriveToPose(objective::getScoringPose, drivetrain);
    }

    public enum Side {
        LEFT,
        RIGHT
    }

    // Starting facing the driver station, moving counterclockwise
    public record CoralObjective(int branchId, Level level){
        public static final CoralObjective NONE = new CoralObjective(0, Level.L1);

        public Pose2d getScoringPose(){
            return DriverAssistConstants.getReefPositions()[branchId];
        }

        public int getFace(){
            return (branchId) / 2;
        }

        public int getAlgaeLevel(){
            // starting with (0, 1) = 3, then (2, 3) = 2, each reef alternates level between 2 and 3
            return (branchId)/2 % 2 == 0 ? 3 : 2;
        }

        public CoralObjective updateBranchId(int branchId){
            return new CoralObjective(branchId, level);
        }

        public CoralObjective updateLevel(Level level){
            return new CoralObjective(branchId, level);
        }
    }
}
