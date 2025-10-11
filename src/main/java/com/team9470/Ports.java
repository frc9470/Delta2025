package com.team9470;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
    /**
     * Motor IDs
     */

    //done
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(11, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(12, "rio");

    //placeholder
    public static final CanDeviceId CANdle = new CanDeviceId(20, "rio");

    //placeholders:
    public static final CanDeviceId INDEXER_1 = new CanDeviceId(0, "rio");
    public static final CanDeviceId INDEXER_2 = new CanDeviceId(1, "rio");

    // TODO: Replace placeholders.
    public static final CanDeviceId ARM_ANGLE = new CanDeviceId(0, "rio");
    public static final CanDeviceId ARM_ROLLERS = new CanDeviceId(1, "rio");

    //done
    public static final CanDeviceId INTAKE_ARM = new CanDeviceId(9, "rio");
    public static final CanDeviceId INTAKE_ROLLERS = new CanDeviceId(10, "rio");

    /**
     * Sensor IDs
     */
    public static final int INDEXER_SENSOR = 1;
}
