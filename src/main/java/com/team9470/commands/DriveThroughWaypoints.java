package com.team9470.commands;

import com.team9470.TunerConstants;
import com.team9470.subsystems.Swerve;
import com.team9470.util.LogUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives the robot through a series of {@link Pose2d} waypoints without fully stopping at each point.
 * The command advances to the next waypoint once the robot is within a configurable handoff radius,
 * allowing for a continuous sweeping motion while still leveraging the {@link DriveToPose} alignment
 * logic for individual targets.
 */
public class DriveThroughWaypoints extends Command {
    private static final double DEFAULT_HANDOFF_RADIUS_METERS = 0.35;

    private final Swerve drivetrain;
    private final Pose2d[] waypoints;
    private final boolean noInterpolate;
    private final double tolerance;
    private final double handoffRadius;

    private int currentIndex = 0;
    private Pose2d currentTarget;

    private final TrapezoidProfile.Constraints translationConstraints =
            new TrapezoidProfile.Constraints(TunerConstants.maxVelocity, TunerConstants.maxAcceleration);
    private final TrapezoidProfile.Constraints rotationConstraints =
            new TrapezoidProfile.Constraints(TunerConstants.maxAngularVelocity, TunerConstants.maxAngularAcceleration);

    private final ProfiledPIDController pidControllerX = new ProfiledPIDController(5, 0, 0, translationConstraints);
    private final ProfiledPIDController pidControllerY = new ProfiledPIDController(5, 0, 0, translationConstraints);
    private final ProfiledPIDController pidControllerOmega = new ProfiledPIDController(7, 0, 0, rotationConstraints);

    public DriveThroughWaypoints(Swerve drivetrain, Pose2d[] waypoints, boolean noInterpolate, double tolerance) {
        this(drivetrain, waypoints, noInterpolate, tolerance, DEFAULT_HANDOFF_RADIUS_METERS);
    }

    public DriveThroughWaypoints(
            Swerve drivetrain, Pose2d[] waypoints, boolean noInterpolate, double tolerance, double handoffRadius) {
        this.drivetrain = drivetrain;
        this.waypoints = waypoints;
        this.noInterpolate = noInterpolate;
        this.tolerance = tolerance;
        this.handoffRadius = handoffRadius;

        pidControllerOmega.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        currentIndex = 0;
        currentTarget = waypoints.length > 0 ? waypoints[0] : null;

        Pose2d currentPose = drivetrain.getPose();
        pidControllerX.reset(currentPose.getX());
        pidControllerY.reset(currentPose.getY());
        pidControllerOmega.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        if (currentTarget == null) {
            return;
        }

        Pose2d robotPose = drivetrain.getPose();
        Pose2d driveTarget = noInterpolate ? currentTarget : DriveToPose.getDriveTarget(robotPose, currentTarget);

        LogUtil.recordPose2d("DriveThroughWaypoints/CurrentTarget", currentTarget);
        LogUtil.recordPose2d("DriveThroughWaypoints/DriveTarget", driveTarget);

        double xSpeed = pidControllerX.calculate(robotPose.getX(), driveTarget.getX());
        double ySpeed = pidControllerY.calculate(robotPose.getY(), driveTarget.getY());
        double thetaSpeed = pidControllerOmega.calculate(
                robotPose.getRotation().getRadians(), driveTarget.getRotation().getRadians());

        double translationMagnitude = Math.hypot(xSpeed, ySpeed);
        if (translationMagnitude > 1e-5) {
            boolean atLastWaypoint = currentIndex >= waypoints.length - 1;
            double desiredMagnitude = atLastWaypoint
                    ? Math.min(translationMagnitude, translationConstraints.maxVelocity)
                    : translationConstraints.maxVelocity;

            double scale = desiredMagnitude / translationMagnitude;
            xSpeed *= scale;
            ySpeed *= scale;
        } else {
            xSpeed = 0;
            ySpeed = 0;
        }

        drivetrain.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed, robotPose.getRotation()));

        if (currentIndex < waypoints.length - 1
                && robotPose.getTranslation().getDistance(currentTarget.getTranslation()) <= handoffRadius) {
            currentIndex++;
            currentTarget = waypoints[currentIndex];
        }
    }

    @Override
    public boolean isFinished() {
        if (currentTarget == null) {
            return true;
        }

        Pose2d robotPose = drivetrain.getPose();
        SmartDashboard.putNumber(
                "DriveThroughWaypoints/TranslationError",
                robotPose.getTranslation().getDistance(currentTarget.getTranslation()));
        SmartDashboard.putNumber(
                "DriveThroughWaypoints/HeadingError",
                Math.abs(robotPose.getRotation().minus(currentTarget.getRotation()).getDegrees()));

        boolean atLastWaypoint = currentIndex >= waypoints.length - 1;
        return atLastWaypoint
                && robotPose.getTranslation().getDistance(currentTarget.getTranslation()) <= 0.01 * tolerance
                && Math.abs(robotPose.getRotation().minus(currentTarget.getRotation()).getDegrees()) <= 1 * tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}

