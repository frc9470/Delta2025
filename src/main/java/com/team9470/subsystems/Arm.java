package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Constants.ArmConstants;
import com.team9470.Ports;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

/**
 * Arm subsystem for scoring/intaking algae and coral in the 2025 FRC REEFSCAPE game.
 * Features motion magic control for precise positioning and roller control for item manipulation.
 * Automatically detects items via current monitoring and appropriately maintains holding force.
 */
public class Arm extends SubsystemBase {
    // --- Hardware ---
    private final TalonFX pivotMotor = TalonFXFactory.createDefaultTalon(Ports.ARM_PIVOT);
    private final TalonFX rollerMotor = TalonFXFactory.createDefaultTalon(Ports.ARM_ROLLERS);

    // --- Control objects ---
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
    private final VoltageOut homingVoltage = new VoltageOut(ArmConstants.HOMING_OUTPUT);

    // --- Status Signals ---
    private final StatusSignal<Angle> positionSignal = pivotMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocitySignal = pivotMotor.getVelocity();
    private final StatusSignal<Current> pivotCurrentSignal = pivotMotor.getStatorCurrent();
    private final StatusSignal<Current> rollerCurrentSignal = rollerMotor.getStatorCurrent();
    private final StatusSignal<Voltage> pivotVoltageSignal = pivotMotor.getMotorVoltage();
    private final StatusSignal<Voltage> rollerVoltageSignal = rollerMotor.getMotorVoltage();

    private final StatusSignal<Double> setpointPositionSignal;

    // --- State tracking ---
    private boolean rollersRunning = false;
    
    // --- Homing logic ---
    private enum HomingState {
        IDLE,
        HOMING,
        HOMED
    }
    private boolean needsHoming = true;
    private HomingState homingState = HomingState.HOMING;
    private Time homingStartTime = Seconds.of(0);

    // --- Target angle ---
    private Angle targetAngle = ArmConstants.HOMING_ANGLE;

    // --- Periodic I/O container ---
    public static class PeriodicIO {
        // Inputs
        public Time timestamp;
        public Angle position;
        public AngularVelocity velocity;

        public Voltage pivotVoltage;
        public Voltage rollerVoltage;

        public Current pivotCurrent;
        public Current rollerCurrent;

        // Outputs / Telemetry
        public Angle goal;
        public HomingState homingState;
        public boolean rollersRunning;

        public double setpointPositionRotations;
    }
    private final PeriodicIO periodicIO = new PeriodicIO();


    public Arm() {
        // Apply motor configurations
        TalonUtil.applyAndCheckConfiguration(pivotMotor, ArmConstants.getPivotConfig());
        TalonUtil.applyAndCheckConfiguration(rollerMotor, ArmConstants.getRollerConfig());

        // Set up sensor status signals
        Frequency refreshRate = Hertz.of(20);
        positionSignal.setUpdateFrequency(refreshRate, 0.1);
        velocitySignal.setUpdateFrequency(refreshRate, 0.1);
        pivotCurrentSignal.setUpdateFrequency(refreshRate, 0.1);
        rollerCurrentSignal.setUpdateFrequency(refreshRate, 0.1);

        setpointPositionSignal = pivotMotor.getClosedLoopReference();
        setpointPositionSignal.setUpdateFrequency(refreshRate, 0.1);

        MusicPlayer.getInstance().addInstrument(pivotMotor);
        MusicPlayer.getInstance().addInstrument(rollerMotor);

    }

    @Override
    public void periodic() {
        readPeriodicInputs();

        // Homing logic
        if (needsHoming) {
            updateHomingLogic();
        } else if (homingState != HomingState.HOMED) {
            homingState = HomingState.IDLE;
        }
        periodicIO.homingState = homingState;

        writePeriodicOutputs();

        logTelemetry();
    }

    private void logTelemetry() {
        // Actual sensor readings
        SmartDashboard.putNumber("Arm/Position_deg", periodicIO.position.in(Degrees));
        SmartDashboard.putNumber("Arm/Velocity_dps", periodicIO.velocity.in(DegreesPerSecond));

        SmartDashboard.putNumber("Arm/PivotCurrent_A", periodicIO.pivotCurrent.in(Amps));
        SmartDashboard.putNumber("Arm/RollerCurrent_A", periodicIO.rollerCurrent.in(Amps));

        SmartDashboard.putNumber("Arm/PivotVoltage_V", periodicIO.pivotVoltage.in(Volts));
        SmartDashboard.putNumber("Arm/RollerVoltage_V", periodicIO.rollerVoltage.in(Volts));

        // Target and goal
        SmartDashboard.putNumber("Arm/TargetAngle_deg", targetAngle.in(Degrees));
        SmartDashboard.putNumber("Arm/Goal_deg", periodicIO.goal.in(Degrees));

        // Homing info
        SmartDashboard.putString("Arm/HomingState", periodicIO.homingState.toString());
        SmartDashboard.putBoolean("Arm/NeedsHoming", needsHoming);

        // Item detection / roller state
        SmartDashboard.putBoolean("Arm/RollersRunning", periodicIO.rollersRunning);
        SmartDashboard.putBoolean("Arm/HoldingItem", isHoldingItem());

        SmartDashboard.putNumber("Arm/Setpoint/Position_Deg", periodicIO.setpointPositionRotations * 360);
    }

    /** Read sensor values into PeriodicIO structure */
    private void readPeriodicInputs() {
        periodicIO.timestamp = Seconds.of(Timer.getFPGATimestamp());
        periodicIO.position = positionSignal.asSupplier().get();
        periodicIO.velocity = velocitySignal.asSupplier().get();
        periodicIO.pivotCurrent = pivotCurrentSignal.asSupplier().get();
        periodicIO.rollerCurrent = rollerCurrentSignal.asSupplier().get();

        periodicIO.pivotVoltage = pivotVoltageSignal.asSupplier().get();
        periodicIO.rollerVoltage = rollerVoltageSignal.asSupplier().get();

        periodicIO.goal = targetAngle;
        periodicIO.rollersRunning = rollersRunning;

        periodicIO.setpointPositionRotations = setpointPositionSignal.asSupplier().get();
    }

    /** Run homing state machine logic */
    private void updateHomingLogic() {
        switch (homingState) {
            case IDLE:
                boolean timeOut = periodicIO.timestamp.minus(homingStartTime).gt(ArmConstants.HOMING_TIMEOUT);
                if (ArmConstants.HOMING_ANGLE.isNear(periodicIO.position, Degrees.of(5)) && timeOut) {
                    homingState = HomingState.HOMING;
                    homingStartTime = periodicIO.timestamp;
                }
                break;
            case HOMING:
                boolean velocityStalled = Math.abs(periodicIO.velocity.in(DegreesPerSecond)) < 0.5;
                boolean currentTooHigh = periodicIO.pivotCurrent.gte(ArmConstants.HOMING_THRESHOLD);
                if (velocityStalled && currentTooHigh) {
                    // Zero the sensor at the homing limit
                    pivotMotor.setPosition(ArmConstants.HOMING_ANGLE);
                    homingState = HomingState.HOMED;
                    needsHoming = false;
                    stopRollers();
                }
                break;
            case HOMED:
                // Stay in HOMED state until explicitly re-homed
                break;
        }
    }

    /** Write outputs to motors */
    private void writePeriodicOutputs() {
        if (homingState == HomingState.HOMING) {
            pivotMotor.setControl(homingVoltage);
            startIntake();
        } else {
            pivotMotor.setControl(
                    motionMagic.withPosition(targetAngle)
                            .withSlot(0)
                            .withEnableFOC(true));
        }
    }

    /** Set the desired target angle for the arm */
    public void setTargetAngle(Angle angle) {
        targetAngle = angle;
        double minDegrees = ArmConstants.MIN_ANGLE.in(Degrees);
        double maxDegrees = ArmConstants.MAX_ANGLE.in(Degrees);
        double clamped = MathUtil.clamp(angle.in(Degrees), minDegrees, maxDegrees);
        targetAngle = Degrees.of(clamped);
    }

    /** Get the current arm angle */
    public Angle getAngle() {
        return periodicIO.position;
    }

    /** Get the current angular velocity */
    public AngularVelocity getAngularVelocity() {
        return periodicIO.velocity;
    }

    /** Get the pivot motor current */
    public Current getPivotCurrent() {
        return periodicIO.pivotCurrent;
    }

    /** Get the roller motor current */
    public Current getRollerCurrent() {
        return periodicIO.rollerCurrent;
    }

    /** Start intake rollers */
    public void startIntake() {
        rollerMotor.setVoltage(ArmConstants.ROLLER_INTAKE_SPEED.in(Volts));
        rollersRunning = true;
    }

    /** Start output rollers */
    public void startOutput() {
        rollerMotor.setVoltage(ArmConstants.ROLLER_OUTPUT_SPEED.in(Volts));
        rollersRunning = true;
    }

    /** Hold item with stalling voltage */
    public void holdItem() {
        rollerMotor.setVoltage(ArmConstants.ROLLER_HOLD_SPEED.in(Volts));
        rollersRunning = true;
    }

    /** Stop rollers */
    public void stopRollers() {
        rollerMotor.stopMotor();
        rollersRunning = false;
    }

    /** Trigger homing sequence */
    public void triggerHoming() {
        needsHoming = true;
        homingState = HomingState.HOMING;
        homingStartTime = Seconds.of(Timer.getFPGATimestamp());;
    }

    /** Check if rollers are running */
    public boolean areRollersRunning() {
        return rollersRunning;
    }

    /** Command to move the arm to the stow angle. */
    public Command stowCommand() {
        return moveCommand(ArmConstants.STOW_ANGLE).withTimeout(.5).andThen(getHomingCommand());
    }

    /** Returns whether the arm is close to the stow position. */
    public boolean isStowed() {
        return getAngle().isNear(ArmConstants.STOW_ANGLE, Degrees.of(3));
    }

    // --- Command Methods ---

    /**
     * Returns a command that moves the arm to the specified angle
     */
    public Command moveCommand(Angle angle) {
        return this.run(() -> setTargetAngle(angle))
                .until(() -> getAngle().isNear(angle, Degrees.of(7)));
    }

    /**
     * Returns a command that starts homing the arm
     */
    public Command getHomingCommand() {
        return this.runOnce(this::triggerHoming)
            .andThen(Commands.waitUntil(() -> homingState == HomingState.HOMED)).finallyDo(this::stopRollers);
    }

    // --- Algae Scoring Commands ---

    public Command algaeBargeBeforeScoringCommand() {
        return moveCommand(ArmConstants.ALGAE_BARGE_BEFORE_SCORING);
    }

    public Command algaeBargeScoringCommand() {
        return moveCommand(ArmConstants.ALGAE_BARGE_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !isHoldingItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    public Command algaeProcessorScoringCommand() {
        return moveCommand(ArmConstants.ALGAE_PROCESSOR_SCORING)
                .andThen(new Command() {
                    @Override
                    public void execute() {
                        startOutput();
                    }
                    @Override
                    public boolean isFinished() {
                        return !isHoldingItem(); // Stop when item is no longer detected
                    }
                    @Override
                    public void end(boolean interrupted) {
                        stopRollers();
                    }
                });
    }

    // --- General Commands ---

    public Command runIntakeCommand() {
        return this.run(this::startIntake);
    }

    public Command runOuttakeCommand() { return this.run(this::startOutput); }

    public Command stopRollersCommand() {
        return this.runOnce(this::stopRollers);
    }

    public Command holdItemCommand() {
        return this.run(this::holdItem);
    }

    public boolean isHoldingItem() {
        try {
            return periodicIO.rollerCurrent.gte(ArmConstants.ITEM_DETECTION_CURRENT);
        } catch (Exception e) {
            return false;
        }
    }
}
