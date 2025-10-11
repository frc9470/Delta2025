package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    /**
     * Motor IDs
     */
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(14, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(15, "rio");

    public static final CanDeviceId CANdle = new CanDeviceId(20, "rio");

    //placeholders:
    public static final CanDeviceId INDEXER_1 = new CanDeviceId(0, "rio");
    public static final CanDeviceId INDEXER_2 = new CanDeviceId(1, "rio");

    public static final CanDeviceId INTAKE_ARM = new CanDeviceId(25, "rio");
    public static final CanDeviceId INTAKE_ROLLERS = new CanDeviceId(26, "rio");

    /**
     * Sensor IDs
     */
    public static final int INDEXER_SENSOR = 1;
    public static final int CORAL_BREAK = 0; // placeholder number, to resolve error on line 30 in Intake.java
}
