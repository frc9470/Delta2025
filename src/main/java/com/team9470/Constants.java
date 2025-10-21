package com.team9470;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.team9470.commands.AutoScoring;
import com.team9470.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static class Global {
        // we want apriltags! see `FieldConstants.java`
        public static final boolean disableHAL = false;
    }

    public static class VisionConstants {
        public static final Transform3d FRONT_LEFT_CAMERA_OFFSET = new Transform3d(Units.Inches.of(+12.290427), Units.Inches.of(10.710), Units.Inches.of(+8.803138),
                new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(-45)));
        public static final Transform3d FRONT_RIGHT_CAMERA_OFFSET = new Transform3d(Units.Inches.of(+12.290427), Units.Inches.of(-10.710), Units.Inches.of(+8.803138),
                new Rotation3d(0, Math.toRadians(-28.125), Math.toRadians(45)));
    }

    public static final class ElevatorConstants {
        public static final Distance DIST_PER_ROTATION =
                Meter.of(.15);
        public static final double rotationsPerMeter = 1.0 / DIST_PER_ROTATION.in(Meters);
        public static final double GEAR_RATIO = 52. / 11;
        public static final double MASS = 3.46;
        public static final double DRUM_RADIUS = DIST_PER_ROTATION.in(Meter) / (2 * Math.PI);

        // Motion config
        public static final LinearVelocity CRUISE_VELOCITY = Units.MetersPerSecond.of(3);
        public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(20);
        public static final double JERK = 0;

        // Homing
        public static final Voltage HOMING_OUTPUT = Units.Volts.of(-2.0);
        public static final Time HOMING_TIMEOUT = Units.Seconds.of(10);

        // Current limits

        public static final Distance HOME_POSITION = Meters.of(0.033);
        public static final Distance L0 = Meters.of(0.0); // INTAKE
        public static final Distance L1 = Meters.of(0.22);
        public static final Distance L2 = Meters.of(0.0);
        public static final Distance L3 = Meters.of(0.36);
        public static final Distance L4 = Meters.of(0.97);
        public static final Distance INTAKE = Meters.of(0);

        public static final Distance STOW_POSITION = Meters.of(0.15);


        public static TalonFXConfiguration ElevatorFXConfig(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(MetersPerSecond) * rotationsPerMeter;
            SmartDashboard.putNumber("Elevator/CruiseVelocity", config.MotionMagic.MotionMagicCruiseVelocity);
            config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(MetersPerSecondPerSecond) * rotationsPerMeter;
            config.MotionMagic.MotionMagicJerk = JERK;
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            config.Slot0.kP = 6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0; // 1
            config.Slot0.kG = 0.26; // 0.5 it's drifting up, 0.3 little too high, 0.28 little too high
            config.Slot0.kS = 0; // 0.05
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }

        public static TalonFXConfiguration ElevatorFXConfigFollower(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40;
            return config;
        }
    }

    public static final class IndexerConstants {
        // Motor control voltages
        public static final Voltage INDEXER_SPEED = Volts.of(-5.0);
        public static final Voltage INDEXER_REVERSE_SPEED = Volts.of(2.0);
        public static final Voltage INDEXER_HOLD_SPEED = Volts.of(0.1);
        
        // Current thresholds for coral detection
        public static final Current CORAL_DETECTION_CURRENT = Amps.of(8.0);
        
        // Timeout for coral detection debouncing (rising-edge filter)
        public static final double CORAL_DETECTION_TIMEOUT = 0.1; // seconds
        
        public static TalonFXConfiguration getMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 25;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }
    }

    public static final class IntakeConstants {
        // Range of motion: when fully down, is at -149... when up
        // 28 is the center of gravity; 118 is horizontal, 149 is fully down

        // Arm positions
        public static final Angle RETRACTED_ANGLE = Degrees.of(90); // Retracted position, originally 118 but hits echain
        public static final Angle DOWN_ANGLE = Degrees.of(-32); // Down position for ground intake
        public static final Angle HOMING_ANGLE = Degrees.of(118); // Use down position for homing

        // Motion control
        public static final double CRUISE_VELOCITY = 2; // degrees per second
        public static final double ACCELERATION = 15; // degrees per second squared
        public static final double JERK = 0;
        
        // Homing
        public static final Current HOMING_THRESHOLD = Amps.of(40); // Current threshold for homing
        public static final Voltage HOMING_OUTPUT = Volts.of(2.0); // Homing voltage
        public static final Time HOMING_TIMEOUT = Seconds.of(10); // Homing timeout
        
        // Roller control
        public static final Voltage ROLLER_SPEED = Volts.of(6.0); // Roller intake speed
        public static final Voltage ROLLER_REVERSE_SPEED = Volts.of(-2.0); // Roller reverse speed
        
        // Physical properties
        // TODO: Tested to be 28.787878, originally 10
        public static final double GEAR_RATIO = 28.78; // Arm gear ratio
        public static final Mass ARM_MASS = Kilogram.of(2.0); // Arm mass in kg
        public static final Distance ARM_LENGTH = Meter.of(0.3); // Arm length in meters
        public static final Angle MIN_ANGLE = Degrees.of(-95); // Minimum angle
        public static final Angle MAX_ANGLE = Degrees.of(5); // Maximum angle
        
        public static TalonFXConfiguration getArmConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
            config.MotionMagic.MotionMagicAcceleration = ACCELERATION;
            config.MotionMagic.MotionMagicJerk = JERK;
            config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            config.Slot0.kP = 15;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kG = 0.31;
            config.Slot0.kS = 0.0;
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }
        
        public static TalonFXConfiguration getRollerConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 25;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }
    }

    public static final class ArmConstants {
        // Intake angles
        public static final Angle STOW_ANGLE = Degrees.of(-88);

        public static final Angle CORAL_HANDOFF_PREP_ANGLE = Degrees.of(-88);

        public static final Angle ALGAE_GROUND_INTAKE_ANGLE = Degrees.of(-30);
        public static final Angle ALGAE_REEF_INTAKE_ANGLE = Degrees.of(0);

        // Coral scoring angles
        public static final Angle CORAL_L4_BEFORE_SCORING = Degrees.of(60);
        public static final Angle CORAL_L4_RELEASE = Degrees.of(10);
        public static final Angle CORAL_L4_SCORING = Degrees.of(0);

        public static final Angle CORAL_L3_BEFORE_SCORING = Degrees.of(60);
        public static final Angle CORAL_L3_RELEASE = Degrees.of(30);
        public static final Angle CORAL_L3_SCORING = Degrees.of(0);

        public static final Angle CORAL_L2_BEFORE_SCORING = Degrees.of(60);
        public static final Angle CORAL_L2_RELEASE = Degrees.of(30);
        public static final Angle CORAL_L2_SCORING = Degrees.of(0);

        public static final Angle CORAL_L1_BEFORE_SCORING = Degrees.of(-30);
        public static final Angle CORAL_L1_SCORING = Degrees.of(-30);

        // Algae scoring angles / poses
        public static final Angle ALGAE_HOLD_ANGLE = Degrees.of(80);
        public static final Angle ALGAE_BARGE_BEFORE_SCORING = Degrees.of(110);
        public static final Angle ALGAE_BARGE_SCORING = Degrees.of(80);
        public static final Angle ALGAE_PROCESSOR_BEFORE_SCORING = Degrees.of(-45);
        public static final Angle ALGAE_PROCESSOR_SCORING = Degrees.of(-40);

        // Homing
        public static final Angle HOMING_ANGLE = Degrees.of(-88.496888);
        public static final Current HOMING_THRESHOLD = Amps.of(20);
        public static final Voltage HOMING_OUTPUT = Volts.of(-2.0);
        public static final Time HOMING_TIMEOUT = Seconds.of(10);

        // Roller control
        public static final Voltage ROLLER_INTAKE_SPEED = Volts.of(6.0);
        public static final Voltage ROLLER_OUTPUT_SPEED = Volts.of(-6.0);
        public static final Voltage ROLLER_HOLD_SPEED = Volts.of(0.5); // Stalls to hold items
        
        // Item detection
        public static final Current ITEM_DETECTION_CURRENT = Amps.of(30.0);
        public static final Time INTAKE_TIMEOUT = Seconds.of(1);

        // Physical properties
        public static final double GEAR_RATIO = (44.0 / 16.0) * (44.0 / 18.0) * (60.0 / 12.0);
        // TODO: Verify values.
        public static final Mass ARM_MASS = Kilogram.of(3.0);
        public static final Distance ARM_LENGTH = Meter.of(0.4);
        public static final Angle MIN_ANGLE = Degrees.of(-88.496888);
        public static final Angle MAX_ANGLE = Degrees.of(95);
        
        public static TalonFXConfiguration getPivotConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotionMagic.MotionMagicCruiseVelocity = 7;
            config.MotionMagic.MotionMagicAcceleration = 20;
            config.MotionMagic.MotionMagicJerk = 0;
            config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            config.Slot0.kP = 20;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 1.0;
            config.Slot0.kG = 0.57;
            config.Slot0.kS = 0.0;
            config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 40;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            return config;
        }
        
        public static TalonFXConfiguration getRollerConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 50;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }
    }

    public static final class DriverAssistConstants {
        public static final Distance centerX = Meters.of(4.47675);
        public static final Distance centerY = Meters.of(4.0259);
        public static final Distance radius = Meters.of(1.51);
        public static final Distance pathRadius = Meters.of(1.51);
        public static final Distance pipeDistance = Meters.of(0.1651);
        public static final Distance fieldLength = Meters.of(17.548225);
        public static final Distance OUTWARD_OFFSET = Inches.of(-5); // distance away from reef
        public static final Distance MANUAL_LATERAL_OFFSET = Inches.of(1); // manual left/right shift

        public static Pose2d[] getReefPositions() {
            Pose2d[] reefPositions = new Pose2d[12];

            for (int i = 0; i < 6; i++) {
                // Hex geometry angles
                double angle1 = Math.PI / 6 + Math.PI / 3 * i;
                double angle2 = Math.PI / 6 + Math.PI / 3 * (i + 1);

                // Points on the hexagon
                double x1 = centerX.in(Meters) + pathRadius.in(Meters) * Math.cos(angle1);
                double y1 = centerY.in(Meters) + pathRadius.in(Meters) * Math.sin(angle1);
                double x2 = centerX.in(Meters) + pathRadius.in(Meters) * Math.cos(angle2);
                double y2 = centerY.in(Meters) + pathRadius.in(Meters) * Math.sin(angle2);

                // Midpoint of each reef face
                double midX = (x1 + x2) / 2;
                double midY = (y1 + y2) / 2;

                // Facing direction (normal of the face)
                double faceAngle = Math.atan2(midY - centerY.in(Meters), midX - centerX.in(Meters)) + Math.PI;

                // Offsets
                double lateralOffset = pipeDistance.in(Meters); // left/right branch separation
                double outwardOffset = OUTWARD_OFFSET.in(Meters); // distance away from reef
                double manualLateralOffset = MANUAL_LATERAL_OFFSET.in(Meters); // manual left/right shift

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
}