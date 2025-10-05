package com.team9470.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team9470.Ports;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.team9470.Constants.ArmConstants;

import static edu.wpi.first.units.Units.Seconds;

public class Arm extends SubsystemBase {
    private final TalonFX pivotMotor = TalonFXFactory.createDefaultTalon(Ports.ARM_ANGLE);
    private final TalonFX rollerMotor = TalonFXFactory.createDefaultTalon(Ports.ARM_ROLLERS);

    private final DigitalInput cradleSensor = new DigitalInput(Ports.CRADLE_BREAK);

    /** STATUS SIGNALS for current monitoring */
    private final StatusSignal<Current> pivotMotorCurrentSignal = pivotMotor.getStatorCurrent();
    private final StatusSignal<Current> rollerMotorCurrentSignal = rollerMotor.getStatorCurrent();

    private boolean rollersRunning = false;
    private boolean cradleCoral = false;

    // ONE of the two will be true:
    private boolean holdingCoral = false;
    private boolean holdingAlgae = false;

    // --- Homing logic ---
    private enum HomingState {
        IDLE,
        HOMING,
        HOMED
    };
    private boolean needsHoming = true;
    // Should start homing at the beginning of the match.
    private HomingState homingState = HomingState.HOMING;
    private Time homingStartTime = Seconds.of(0);

    // --- Target angle ---
    // private Angle targetAngle =

    // TODO: Construct simulation objects (PeriodicIO, Ligament Mechanism, etc.

    public Arm() {
        // Apply motor configurations.
        TalonUtil.applyAndCheckConfiguration(pivotMotor, ArmConstants.getPivotConfig());
        TalonUtil.applyAndCheckConfiguration(rollerMotor, ArmConstants.getRollerConfig());

        // Set up status signals for current monitoring.
        pivotMotorCurrentSignal.setUpdateFrequency(50, 0.1);
        rollerMotorCurrentSignal.setUpdateFrequency(50, 0.1);

        // Set default command to stop rollers.
    }

    @Override
    public void periodic() {
        updateItemDetection();

        // Inverted logic, so will be true when
        cradleCoral = !cradleSensor.get();
    }

    private void updateItemDetection() {
        // Get motor reading.
        Current rollerMotorCurrent = rollerMotorCurrentSignal.asSupplier().get();

        // Check if the motor is drawing high current.
        boolean rollerHighCurrent = rollerMotorCurrent.gte(ArmConstants.ITEM_DETECTION_CURRENT);

        // If high current, there is an item.
        if (rollerHighCurrent) {

        }
    }
}
