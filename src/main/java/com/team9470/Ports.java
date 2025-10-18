package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    /**
     * Motor IDs
     */

    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(17, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(18, "rio");

    public static final CanDeviceId INDEXER_1 = new CanDeviceId(15, "rio");
    public static final CanDeviceId INDEXER_2 = new CanDeviceId(16, "rio");

    public static final CanDeviceId ARM_ANGLE = new CanDeviceId(19, "rio"); // 19
    public static final CanDeviceId ARM_ROLLERS = new CanDeviceId(20, "rio"); // 20

    public static final CanDeviceId INTAKE_ARM = new CanDeviceId(14, "rio");
    public static final CanDeviceId INTAKE_ROLLERS = new CanDeviceId(13, "rio");

    /**
     * Sensor IDs
     */
    public static final int INDEXER_SENSOR = 1;
    public static final int CORAL_BREAK = 0;
}
