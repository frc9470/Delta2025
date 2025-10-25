package com.team9470.constants;

import com.team9470.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * Precomputed geometry used by driver-assist features.
 */
public final class DriverAssistConstants {
    private DriverAssistConstants() {}

    public static final Distance centerX = Units.Meters.of(4.47675);
    public static final Distance centerY = Units.Meters.of(4.0259);
    public static final Distance radius = Units.Meters.of(1.51);
    public static final Distance pathRadius = Units.Meters.of(1.51);
    public static final Distance pipeDistance = Units.Meters.of(0.1651);
    public static final Distance fieldLength = Units.Meters.of(17.548225);
    public static final Distance OUTWARD_OFFSET = Units.Inches.of(-5); // distance away from reef
    public static final Distance MANUAL_LATERAL_OFFSET = Units.Inches.of(0); // manual left/right shift

    public static Pose2d[] getReefPositions() {
        Pose2d[] reefPositions = new Pose2d[12];

        for (int i = 0; i < 6; i++) {
            // Hex geometry angles
            double angle1 = Math.PI / 6 + Math.PI / 3 * i;
            double angle2 = Math.PI / 6 + Math.PI / 3 * (i + 1);

            // Points on the hexagon
            double x1 = centerX.in(Units.Meters) + pathRadius.in(Units.Meters) * Math.cos(angle1);
            double y1 = centerY.in(Units.Meters) + pathRadius.in(Units.Meters) * Math.sin(angle1);
            double x2 = centerX.in(Units.Meters) + pathRadius.in(Units.Meters) * Math.cos(angle2);
            double y2 = centerY.in(Units.Meters) + pathRadius.in(Units.Meters) * Math.sin(angle2);

            // Midpoint of each reef face
            double midX = (x1 + x2) / 2;
            double midY = (y1 + y2) / 2;

            // Facing direction (normal of the face)
            double faceAngle = Math.atan2(midY - centerY.in(Units.Meters), midX - centerX.in(Units.Meters)) + Math.PI;

            // Offsets
            double lateralOffset = pipeDistance.in(Units.Meters); // left/right branch separation
            double outwardOffset = OUTWARD_OFFSET.in(Units.Meters); // distance away from reef
            double manualLateralOffset = MANUAL_LATERAL_OFFSET.in(Units.Meters); // manual left/right shift

            // Left branch
            double leftX = midX - lateralOffset * Math.sin(Math.PI / 3 * (i - 2));
            double leftY = midY + lateralOffset * Math.cos(Math.PI / 3 * (i - 2));
            leftX += outwardOffset * Math.cos(faceAngle);
            leftY += outwardOffset * Math.sin(faceAngle);
            // Manual lateral offset (perpendicular to faceAngle)
            leftX += manualLateralOffset * Math.sin(faceAngle);
            leftY -= manualLateralOffset * Math.cos(faceAngle);

            // Right branch
            double rightX = midX + lateralOffset * Math.sin(Math.PI / 3 * (i - 2));
            double rightY = midY - lateralOffset * Math.cos(Math.PI / 3 * (i - 2));
            rightX += outwardOffset * Math.cos(faceAngle);
            rightY += outwardOffset * Math.sin(faceAngle);
            // Manual lateral offset (perpendicular to faceAngle)
            rightX += manualLateralOffset * Math.sin(faceAngle);
            rightY -= manualLateralOffset * Math.cos(faceAngle);

            // Store both poses, facing away from reef
            reefPositions[2 * i] = AllianceFlipUtil.apply(
                    new Pose2d(leftX, leftY, new Rotation2d(faceAngle))
            );
            reefPositions[2 * i + 1] = AllianceFlipUtil.apply(
                    new Pose2d(rightX, rightY, new Rotation2d(faceAngle))
            );
        }

        // Rotate positions 4 spots clockwise to match field layout
        Pose2d[] rotatedPositions = new Pose2d[12];
        for (int i = 0; i < 12; i++) {
            int newIndex = (i + 8 + 12) % 12;
            rotatedPositions[newIndex] = reefPositions[i];
        }

        return rotatedPositions;
    }
}
