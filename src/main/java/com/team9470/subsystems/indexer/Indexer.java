package com.team9470.subsystems.indexer;

import static com.team9470.subsystems.indexer.IndexerConstants.CORAL_DETECTION_CURRENT;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.subsystems.MusicPlayer;
import com.team9470.Ports;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Indexer subsystem for handling coral in the 2025 FRC Reefscape game.
 * Uses two motors running in opposite directions to move coral through the system.
 * Detects coral presence using current monitoring and beam break sensors.
 */
public class Indexer extends SubsystemBase {

    private final TalonFX indexerMotor1 = TalonFXFactory.createDefaultTalon(Ports.INDEXER_1);
    private final TalonFX indexerMotor2 = TalonFXFactory.createDefaultTalon(Ports.INDEXER_2);
    

    private final DigitalInput coralSensor = new DigitalInput(Ports.CRADLE_BREAK); //beam break

    private final Debouncer coralDebouncer = new Debouncer(
        IndexerConstants.CORAL_DETECTION_TIMEOUT,
        DebounceType.kRising
    );

    private boolean coralInCradle;
    
    /** STATUS SIGNALS for current monitoring */
    private final StatusSignal<Current> motor1CurrentSignal = indexerMotor1.getStatorCurrent();
    private final StatusSignal<Current> motor2CurrentSignal = indexerMotor2.getStatorCurrent();
    
    public Indexer() {
        // Apply motor configurations
        TalonUtil.applyAndCheckConfiguration(indexerMotor1, IndexerConstants.getMotorConfig());
        TalonUtil.applyAndCheckConfiguration(indexerMotor2, IndexerConstants.getMotorConfig());

        // Set up status signals for current monitoring
        motor1CurrentSignal.setUpdateFrequency(50, 0.1);
        motor2CurrentSignal.setUpdateFrequency(50, 0.1);

        coralInCradle = readCradleBeamBreak();

        // Set default command to stop the indexer
        setDefaultCommand(stopCommand());
        MusicPlayer.getInstance().addInstrument(indexerMotor1);
        MusicPlayer.getInstance().addInstrument(indexerMotor2);
    }

    @Override
    public void periodic() {
        coralInCradle = coralDebouncer.calculate(readCradleBeamBreak());
        // Push values to SmartDashboard
        logTelemetry();
    }

    private void logTelemetry() {
        SmartDashboard.putBoolean("Indexer/hasCoral", hasCoral());
        SmartDashboard.putNumber("Indexer/Current1", indexerMotor1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Current2", indexerMotor2.getStatorCurrent().getValueAsDouble());
    }

    private boolean readCradleBeamBreak() {
        // The sensor returns false when blocked, so invert to represent coral present.
        return !coralSensor.get();
    }

    /**
     * Starts the indexer with both motors running in opposite directions
     */
    public void startIndexer() {
        indexerMotor1.setVoltage(IndexerConstants.INDEXER_SPEED.in(Volts));
        indexerMotor2.setVoltage(-IndexerConstants.INDEXER_SPEED.in(Volts));
    }

    public void unjamIndexer() {
        indexerMotor1.setVoltage(-IndexerConstants.INDEXER_SPEED.in(Volts));
        indexerMotor2.setVoltage(-IndexerConstants.INDEXER_SPEED.in(Volts));
    }


    /**
     * Stops the indexer by stopping both motors
     */
    public void stopIndexer() {
        indexerMotor1.stopMotor();
        indexerMotor2.stopMotor();
    }

    /**
     * Runs the indexer in reverse (both motors opposite to normal direction)
     */
    public void reverseIndexer() {
        indexerMotor1.setVoltage(IndexerConstants.INDEXER_REVERSE_SPEED.in(Volts));
        indexerMotor2.setVoltage(-IndexerConstants.INDEXER_REVERSE_SPEED.in(Volts));
    }

    /**
     * Determines whether a coral is currently in the indexer
     * Uses debounced detection to prevent false positives
     * 
     * @return true if coral is detected in the indexer
     */
    public boolean hasCoral() {
        return coralInCradle;
    }
    /**
     * Outputs coral into robot, assuming coral is being held right now
     * @return command
     */
    public Command outputCommand() {
        return this.run(this::startIndexer)
                .until(() -> !hasCoral())
                .andThen(this::stopIndexer);
    }

    /**
     * Returns a command that runs the indexer continuously
     */
    public Command runCommand() {
        return this.run(() -> {

                if(indexerMotor1.getStatorCurrent().getValue().lte(CORAL_DETECTION_CURRENT) || indexerMotor2.getStatorCurrent().getValue().lte(CORAL_DETECTION_CURRENT))
                    this.unjamIndexer();
                else
                    this.startIndexer();
            }
        );

    }

    /**
     * Returns a command that runs the indexer in reverse
     */
    public Command reverseCommand() {
        return this.run(this::reverseIndexer);
    }

    public Command stopCommand() {
        return this.run(this::stopIndexer);
    }
}
