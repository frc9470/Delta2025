package com.team9470;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team9470.commands.AutoScoring;
import com.team9470.commands.Autos;
import com.team9470.commands.WheelRadiusCharacterization;
import com.team9470.subsystems.MusicPlayer;
import com.team9470.subsystems.Superstructure;
import com.team9470.subsystems.Swerve;
import com.team9470.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

// TODO: Remove after debug done.

import static com.team9470.Constants.*;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Drivetrain and related commands remain unchanged
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final Swerve drivetrain = TunerConstants.createDrivetrain();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    // ---------------- MECHANISM2D --------------------
    private final Mechanism2d mech = new Mechanism2d(5, 10);

    // Create the Superstructure instead of separate subsystems.
    private final Superstructure superstructure = new Superstructure(mech, drivetrain);
    private final AutoScoring autoScoring = new AutoScoring(drivetrain);

    // ----------------      VISION     --------------------
    private final Vision vision = Vision.getInstance();

    // ---------------- AUTONOMOUS --------------------
    private final Autos autos = new Autos(superstructure, drivetrain);
    private final AutoChooser autoChooser = new AutoChooser();

    CommandXboxController xbox = new CommandXboxController(0);
    Joystick buttonBoard = new Joystick(1);

    public RobotContainer() {
        DataLogManager.start();
        configureBindings();

        autoChooser.addRoutine("1CMN", autos::getOneCoralMiddleAutoNormal);
        autoChooser.addRoutine("1CMC", autos::getOneCoralMiddleAutoChoreo);
        autoChooser.addRoutine("1CMA", autos::getOneCoralMiddleAutoAlign);
        autoChooser.addRoutine("5CTA", autos::getFiveCoralTopAutoAlign);
        autoChooser.addRoutine("5CBA", autos::getFiveCoralBottomAutoAlign);
        autoChooser.addRoutine("5CBA-NW", autos::getFiveCoralBottomAutoAlignNoWait);
        autoChooser.addRoutine("5CTA-NW", autos::getFiveCoralTopAutoAlignNoWait);
        autoChooser.select("2C Test");
        SmartDashboard.putData("AutoChooser", autoChooser);
        SmartDashboard.putData("Mechanism", mech);
        SmartDashboard.putData(
                "Drive/Commands/Characterize Wheel Radius (CW)",
                new WheelRadiusCharacterization(
                        drivetrain, WheelRadiusCharacterization.Direction.CLOCKWISE));
        SmartDashboard.putData(
                "Drive/Commands/Characterize Wheel Radius (CCW)",
                new WheelRadiusCharacterization(
                        drivetrain, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));

        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    public void periodic(){
        drivetrain.periodic();

    }

    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-xbox.getLeftY() * MaxSpeed)
                                .withVelocityY(-xbox.getLeftX() * MaxSpeed)
                                .withRotationalRate(-xbox.getRightX() * MaxAngularRate)
                )
        );

//        xbox.rightTrigger()
//                .whileTrue(autoScoring.autoScore(superstructure)).onFalse(superstructure.getElevator().L0());
        // TODO: Remove / verify.
        xbox.rightTrigger()
            .onTrue(
                superstructure.moveElevatorToHeight(ElevatorConstants.STOW_POSITION)
                    .andThen(superstructure.intakeCoralGround())
            ).onFalse(
                superstructure.stopIntakeGround()
            );

//        xbox.a().whileTrue(superstructure.coralCradlePickup());
//
//        xbox.b().whileTrue(superstructure.armIntake()).onFalse(superstructure.armStop());

        xbox.povDown()
            .whileTrue(superstructure.coralCradlePickup());

        xbox.povUp()
            .whileTrue(superstructure.armIntake())
                .onFalse(superstructure.armStop());

        xbox.povRight()
            .onTrue(superstructure.stow());

        xbox.povLeft()
            .whileTrue(superstructure.armOuttake())
                .onFalse(superstructure.armStop());

        // TODO: Implement auto-align.
        xbox.a()
            .toggleOnTrue(superstructure.prepareLevel(Superstructure.Level.L1)
                    .finallyDo(superstructure::stow));

        xbox.b()
                .toggleOnTrue(superstructure.prepareLevel(Superstructure.Level.L2)
                        .finallyDo(superstructure::stow));

        xbox.x()
                .toggleOnTrue(superstructure.prepareLevel(Superstructure.Level.L3)
                        .finallyDo(superstructure::stow));

        xbox.y()
                .toggleOnTrue(superstructure.prepareLevel(Superstructure.Level.L4)
                        .finallyDo(superstructure::stow));

        xbox.leftBumper()
                .onTrue(autoScoring.autoAlign(superstructure, AutoScoring.Side.LEFT));

        xbox.leftTrigger()
                .onTrue(autoScoring.autoAlign(superstructure, AutoScoring.Side.RIGHT));

        xbox.rightBumper()
                .whileTrue(
                        superstructure.scoreHeldPiece()
                );


    }
}
