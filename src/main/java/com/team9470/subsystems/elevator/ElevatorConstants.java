package com.team9470.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

/**
 * Constants and configuration helpers for the elevator subsystem.
 */
public final class ElevatorConstants {
    private ElevatorConstants() {}

    public static final Distance DIST_PER_ROTATION = Meter.of(.15);
    public static final double rotationsPerMeter = 1.0 / DIST_PER_ROTATION.in(Meters);
    public static final double GEAR_RATIO = 52.0 / 11.0;
    public static final double MASS = 3.46;
    public static final double DRUM_RADIUS = DIST_PER_ROTATION.in(Meter) / (2 * Math.PI);

    // Arm safety
    public static final Angle ARM_MIN_ANGLE_FOR_FULL_LOWERING = Units.Degrees.of(-50);
    public static final Angle ARM_STRAIGHT_DOWN_TOLERANCE = Units.Degrees.of(5);

    // Motion config
    public static final LinearVelocity CRUISE_VELOCITY = Units.MetersPerSecond.of(3);
    public static final LinearAcceleration ACCELERATION = MetersPerSecondPerSecond.of(20);
    public static final double JERK = 0;

    // Homing
    public static final Voltage HOMING_OUTPUT = Volts.of(-2.0);
    public static final Time HOMING_TIMEOUT = Seconds.of(10);

    // Positions
    public static final Distance HOME_POSITION = Units.Meters.of(0.033);
    public static final Distance L0 = Units.Meters.of(0.0); // INTAKE
    public static final Distance L1 = Units.Meters.of(0.29);
    public static final Distance L2 = Units.Meters.of(0.0);
    public static final Distance L3 = Units.Meters.of(0.36);
    public static final Distance L4 = Units.Meters.of(0.97);
    public static final Distance INTAKE = Units.Meters.of(0);


    public static final Distance ALGAE_L1 = Units.Meters.of(.0);
    public static final Distance ALGAE_L2 = Units.Meters.of(.33);
    public static final Distance ALGAE_L3 = Units.Meters.of(.68);
    public static final Distance ALGAE_L4 = Units.Meters.of(1.2);

    public static final Distance STOW_POSITION = Units.Meters.of(0.2);

    public static TalonFXConfiguration ElevatorFXConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotionMagic.MotionMagicCruiseVelocity =
                CRUISE_VELOCITY.in(MetersPerSecond) * rotationsPerMeter;
        SmartDashboard.putNumber("Elevator/CruiseVelocity", config.MotionMagic.MotionMagicCruiseVelocity);
        config.MotionMagic.MotionMagicAcceleration =
                ACCELERATION.in(MetersPerSecondPerSecond) * rotationsPerMeter;
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

    public static TalonFXConfiguration ElevatorFXConfigFollower() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        return config;
    }
}
