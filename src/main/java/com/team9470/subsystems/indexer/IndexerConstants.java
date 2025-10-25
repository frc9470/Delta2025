package com.team9470.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Constants for the indexer subsystem.
 */
public final class IndexerConstants {
    private IndexerConstants() {}

    // Motor control voltages
    public static final Voltage INDEXER_SPEED = Units.Volts.of(-5.0);
    public static final Voltage INDEXER_REVERSE_SPEED = Units.Volts.of(2.0);
    public static final Voltage INDEXER_HOLD_SPEED = Units.Volts.of(0.1);

    // Current thresholds for coral detection
    public static final Current CORAL_DETECTION_CURRENT = Units.Amps.of(20.0);

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
