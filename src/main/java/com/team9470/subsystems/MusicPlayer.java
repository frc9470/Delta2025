package com.team9470.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import lombok.Getter;

public class MusicPlayer {

    @Getter
    private static final MusicPlayer instance = new MusicPlayer();
    Orchestra orchestra = new Orchestra();

    private MusicPlayer(){
        // Attempt to load the chrp
        var status = orchestra.loadMusic("megolovania.chrp");

        if (!status.isOK()) {
            System.out.println("Failed to load music: " + status);
        } else {
            System.out.println("Music loaded successfully.");
        }
    }

    public void addInstrument(ParentDevice motor) {
        orchestra.addInstrument(motor);
    }

    public void play() {
        System.out.println("Playing...");
        orchestra.play();
    }
}
