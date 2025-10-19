package com.team9470.subsystems.vision;

import org.junit.jupiter.api.Test;

import java.util.Collections;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

class VisionDeviceTest {

    @Test
    void hasUsableTargetsHandlesEmptyList() {
        assertDoesNotThrow(() -> VisionDevice.hasUsableTargets(Collections.emptyList()));
        assertFalse(VisionDevice.hasUsableTargets(Collections.emptyList()));
    }
}
