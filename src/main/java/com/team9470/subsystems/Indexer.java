package com.team9470.subsystems;

import static com.team9470.Constants.IndexerConstants.CORAL_DETECTION_CURRENT;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.DelayedBoolean;
import com.team9470.Constants.IndexerConstants;
import com.team9470.Ports;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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
    
    /** STATUS SIGNALS for current monitoring */
    private final StatusSignal<Current> motor1CurrentSignal = indexerMotor1.getStatorCurrent();
    private final StatusSignal<Current> motor2CurrentSignal = indexerMotor2.getStatorCurrent();
    
    /** Delayed boolean for coral detection debouncing */
    private final DelayedBoolean coralDetected = new DelayedBoolean(
        Timer.getFPGATimestamp(), 
        IndexerConstants.CORAL_DETECTION_TIMEOUT, 
        false
    );
    
    /** State tracking */
    private boolean isRunning = false;
    private boolean coralPresent = false;

    public Indexer() {
        // Apply motor configurations
        TalonUtil.applyAndCheckConfiguration(indexerMotor1, IndexerConstants.getMotorConfig());
        TalonUtil.applyAndCheckConfiguration(indexerMotor2, IndexerConstants.getMotorConfig());
        
        // Set up status signals for current monitoring
        motor1CurrentSignal.setUpdateFrequency(50, 0.1);
        motor2CurrentSignal.setUpdateFrequency(50, 0.1);
        
        // Set default command to stop the indexer
        setDefaultCommand(stopCommand());
    }

    @Override
    public void periodic() {
        // Update delayed boolean for debouncing
        coralDetected.update(Timer.getFPGATimestamp(), coralPresent);

        // Push values to SmartDashboard
        logTelemetry();
    }

    private void logTelemetry() {
        SmartDashboard.putBoolean("Indexer/hasCoral", hasCoral());
        SmartDashboard.putNumber("Indexer/Current1", indexerMotor1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Current2", indexerMotor2.getStatorCurrent().getValueAsDouble());
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
        return coralDetected.update(Timer.getFPGATimestamp(), coralPresent);
    }

    /**
     * Raw beam break state for the cradle. True when a coral is blocking the sensor.
     */
    public boolean isCoralInCradle() {
        return !coralSensor.get();
    }
    /*

    public Command intakeToOutputCommand() {
        return this.run(this::startIndexer)
                .until(this::hasCoral)
                .andThen(this.run(this::startIndexer))
                .until(() -> !hasCoral())
                .andThen(this::stopIndexer);
    }

    public Command intakeAndHoldCommand(){
        return this.run(this::startIndexer)
        .until(this::hasCoral)
        .andThen(this.run(this::holdIndexer));
    }
    */

    /**
     * Outputs coral into robot, assuming coral is being held right now
     * @return command
     */
    public Command outputCommand(){
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
