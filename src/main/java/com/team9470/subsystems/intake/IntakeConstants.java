package com.team9470.subsystems.intake;

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
 * Constants for the intake subsystem.
 */
public final class IntakeConstants {
    private IntakeConstants() {}

    // Arm positions
    public static final Angle RETRACTED_ANGLE = Units.Degrees.of(90); // Retracted position, originally 118 but hits echain
    public static final Angle DOWN_ANGLE = Units.Degrees.of(-32); // Down position for ground intake
    public static final Angle HOMING_ANGLE = Units.Degrees.of(118); // Use down position for homing

    // Motion control
    public static final double CRUISE_VELOCITY = 2; // degrees per second
    public static final double ACCELERATION = 15; // degrees per second squared
    public static final double JERK = 0;

    // Homing
    public static final Current HOMING_THRESHOLD = Units.Amps.of(40); // Current threshold for homing
    public static final Voltage HOMING_OUTPUT = Units.Volts.of(2.0); // Homing voltage
    public static final Time HOMING_TIMEOUT = Units.Seconds.of(10); // Homing timeout

    // Roller control
    public static final Voltage ROLLER_SPEED = Units.Volts.of(6.0); // Roller intake speed
    public static final Voltage ROLLER_REVERSE_SPEED = Units.Volts.of(-2.0); // Roller reverse speed

    // Physical properties
    // TODO: Tested to be 28.787878, originally 10
    public static final double GEAR_RATIO = 28.78; // Arm gear ratio
    public static final Mass ARM_MASS = Units.Kilogram.of(2.0); // Arm mass in kg
    public static final Distance ARM_LENGTH = Units.Meter.of(0.3); // Arm length in meters
    public static final Angle MIN_ANGLE = Units.Degrees.of(-95); // Minimum angle
    public static final Angle MAX_ANGLE = Units.Degrees.of(5); // Maximum angle

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
