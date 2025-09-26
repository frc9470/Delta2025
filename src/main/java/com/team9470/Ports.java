package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    /**
     * Motor IDs
     */
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(14, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(15, "rio");

    public static final CanDeviceId ALGAE_PIVOT = new CanDeviceId(18, "rio");

    public static final CanDeviceId CORAL_INTAKE = new CanDeviceId(16, "rio");
    public static final CanDeviceId FUNNEL = new CanDeviceId(19, "rio");

    public static final CanDeviceId CANdle = new CanDeviceId(20, "rio");

    public static final CanDeviceId CLIMBER_MAIN = new CanDeviceId(21, "rio");
    public static final CanDeviceId CLIMBER_FOLLOWER = new CanDeviceId(22, "rio");

    public static final CanDeviceId CLIMBER_WHEELS = new CanDeviceId(23, "rio");
    public static final CanDeviceId FUNNEL_CONTROL = new CanDeviceId(24, "rio");

    //placeholders:
    public static final CanDeviceId INDEXER_1 = new CanDeviceId(0, "rio");
    public static final CanDeviceId INDEXER_2 = new CanDeviceId(1, "rio");

    /**
     * Beam Break IDs
     */
    public static final int CORAL_BREAK = 1;
}
