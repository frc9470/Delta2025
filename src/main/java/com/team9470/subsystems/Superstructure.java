package com.team9470.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    private final Elevator elevator;
    private final Indexer indexer;
    private final LEDs leds;

    public Superstructure(Mechanism2d mech) {
        this.elevator = new Elevator(mech);
        this.leds = LEDs.getInstance();
        this.indexer = new Indexer();
    }

    public Command reverseCoral() {
        return indexer.reverseCommand();
    }

    // Stow algae in alternate position (stow location 2) for coral scoring:
    // Raise elevator to L2 and move algae below the coral manipulator.
    public Command algaeUp() {
        return Commands.none();
    }

    public Command algaeDown() {
        return Commands.none();
    }

    public Command raise(int level){
        return elevator.getLevelCommand(level);
    }

    public Command raise(Supplier<Integer> getLevel){
        return elevator.getLevelCommand(getLevel.get());
    }

    
    public Command score() {
        return indexer.outputCommand();
    }

    public Command scoreSlow() {
        return indexer.outputCommand();
    }

    // Trigger the algae arm’s homing routine.
    public Command triggerAlgaeHoming() {
        return Commands.none();
    }

    public Command waitForIntake() {
        return new WaitUntilCommand(indexer::hasCoral);
    }

    public Command funnelOut() {
        return Commands.none();
    }

    public Command climberAction() {
        return Commands.none();
    }

    public boolean hasGamePiece() {
        return indexer.hasCoral();
    }

    // Accessors for the individual subsystems.
    public Elevator getElevator() {
        return elevator;
    }

    public LEDs getLEDs() {
        return leds;
    }

    public Indexer getIndexer() {
        return indexer;
    }

    public Command scoreAndFunnel() {
        return indexer.outputCommand();
    }
}