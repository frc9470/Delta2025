package com.team9470.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * Constants for the arm subsystem.
 */
public final class ArmConstants {
    private ArmConstants() {}

    // Intake angles
    public static final Angle STOW_ANGLE = Units.Degrees.of(-88);

    public static final Angle CORAL_HANDOFF_PREP_ANGLE = Units.Degrees.of(-88);

    public static final Angle ALGAE_GROUND_INTAKE_ANGLE = Units.Degrees.of(-35);
    public static final Angle ALGAE_REEF_INTAKE_ANGLE = Units.Degrees.of(0);

    // Coral scoring angles
    public static final Angle CORAL_L4_BEFORE_SCORING = Units.Degrees.of(60);
    public static final Angle CORAL_L4_RELEASE = Units.Degrees.of(10);
    public static final Angle CORAL_L4_SCORING = Units.Degrees.of(0);

    public static final Angle CORAL_L3_BEFORE_SCORING = Units.Degrees.of(60);
    public static final Angle CORAL_L3_RELEASE = Units.Degrees.of(30);
    public static final Angle CORAL_L3_SCORING = Units.Degrees.of(0);

    public static final Angle CORAL_L2_BEFORE_SCORING = Units.Degrees.of(60);
    public static final Angle CORAL_L2_RELEASE = Units.Degrees.of(30);
    public static final Angle CORAL_L2_SCORING = Units.Degrees.of(10);

    public static final Angle CORAL_L1_BEFORE_SCORING = Units.Degrees.of(-30);
    public static final Angle CORAL_L1_SCORING = Units.Degrees.of(-30);

    // Algae scoring angles / poses
    public static final Angle ALGAE_HOLD_ANGLE = Units.Degrees.of(80);
    public static final Angle ALGAE_BARGE_BEFORE_SCORING = Units.Degrees.of(70);
    public static final Angle ALGAE_BARGE_SCORING = Units.Degrees.of(70);
    public static final Angle ALGAE_PROCESSOR_BEFORE_SCORING = Units.Degrees.of(-45);
    public static final Angle ALGAE_PROCESSOR_SCORING = Units.Degrees.of(-40);

    // Homing
    public static final Angle HOMING_ANGLE = Units.Degrees.of(-88.496888);
    public static final Current HOMING_THRESHOLD = Units.Amps.of(20);
    public static final Voltage HOMING_OUTPUT = Units.Volts.of(-2.0);
    public static final Time HOMING_TIMEOUT = Units.Seconds.of(10);

    // Roller control
    public static final Voltage ROLLER_INTAKE_SPEED = Units.Volts.of(6.0);
    public static final Voltage ROLLER_OUTPUT_SPEED = Units.Volts.of(-8.0);
    public static final Voltage ROLLER_HOLD_SPEED = Units.Volts.of(2.0); // Stalls to hold items

    // Item detection
    public static final Current ITEM_DETECTION_CURRENT = Units.Amps.of(30.0);
    public static final Time INTAKE_TIMEOUT = Units.Seconds.of(1);

    // Physical properties
    public static final double GEAR_RATIO = (44.0 / 16.0) * (44.0 / 18.0) * (60.0 / 12.0);
    public static final Mass ARM_MASS = Units.Kilogram.of(3.0);
    public static final Distance ARM_LENGTH = Units.Meter.of(0.4);
    public static final Angle MIN_ANGLE = Units.Degrees.of(-88.496888);
    public static final Angle MAX_ANGLE = Units.Degrees.of(95);

    public static TalonFXConfiguration getPivotConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicCruiseVelocity = 7;
        config.MotionMagic.MotionMagicAcceleration = 20;
        config.MotionMagic.MotionMagicJerk = 0;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.kP = 20; // 25 WAYY too fast
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
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        return config;
    }
}
