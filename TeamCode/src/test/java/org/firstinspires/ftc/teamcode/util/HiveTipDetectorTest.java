package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.*;

import org.junit.Test;

public class HiveTipDetectorTest {
    private static final long START = 1_000_000_000L;
    private final HiveTipDetector detector = new HiveTipDetector(35, 0);

    private void frame(long ms, boolean visible) {
        detector.observe(0, 10, ms + 1, visible, START + ms * 1_000_000L);
    }

    @Test
    public void freshMissingFramesMustSpanTwoHundredMs() {
        frame(0, true);
        for (int ms = 20; ms <= 200; ms += 20) frame(ms, false);
        assertFalse(detector.tipped());
        frame(220, false);
        assertTrue(detector.tipped());
    }

    @Test
    public void neverSeenTagCannotCountAsTipped() {
        for (int ms = 0; ms <= 400; ms += 20) frame(ms, false);
        assertEquals(HiveTipDetector.State.WAITING_FOR_TAG, detector.state());
    }

    @Test
    public void frozenFrameBecomesUnknownInsteadOfTipped() {
        frame(0, true);
        frame(20, false);
        for (int ms = 40; ms <= 240; ms += 20) {
            detector.observe(0, 10, 21, false, START + ms * 1_000_000L);
        }
        assertEquals(HiveTipDetector.State.UNKNOWN, detector.state());
        assertFalse(detector.tipped());
    }

    @Test
    public void disconnectedCameraBecomesUnknown() {
        frame(0, true);
        detector.update(null, START + 200_000_000L);
        assertEquals(HiveTipDetector.State.UNKNOWN, detector.state());
        assertFalse(detector.tipped());
    }

    @Test
    public void staleFramesCannotConfirmLoss() {
        frame(0, true);
        for (int ms = 20; ms <= 200; ms += 20) {
            detector.observe(0, TurretConfig.maxFrameAgeMs + 1, ms + 1, false,
                    START + ms * 1_000_000L);
        }
        assertEquals(HiveTipDetector.State.UNKNOWN, detector.state());
    }

    @Test
    public void pipelineSwitchInvalidatesShotObservation() {
        frame(0, true);
        detector.observe(1, 10, 21, false, START + 20_000_000L);
        assertEquals(HiveTipDetector.State.UNKNOWN, detector.state());
    }

    @Test
    public void cameraClockResetInvalidatesShotObservation() {
        frame(20, true);
        detector.observe(0, 10, 1, false, START + 40_000_000L);
        assertEquals(HiveTipDetector.State.UNKNOWN, detector.state());
    }

    @Test
    public void longFrameGapCannotBeBridgedByNewMissingFrame() {
        frame(0, true);
        frame(20, false);
        frame(240, false);
        assertEquals(HiveTipDetector.State.UNKNOWN, detector.state());
        assertFalse(detector.tipped());
    }

    @Test
    public void reacquiringTagRestartsMissingDwell() {
        frame(0, true);
        for (int ms = 20; ms <= 180; ms += 20) frame(ms, false);
        frame(200, true);
        for (int ms = 220; ms <= 400; ms += 20) frame(ms, false);
        assertFalse(detector.tipped());
        frame(420, false);
        assertTrue(detector.tipped());
    }

    @Test
    public void briefUnavailableDataRestartsMissingDwell() {
        frame(0, true);
        for (int ms = 20; ms <= 160; ms += 20) frame(ms, false);
        detector.update(null, START + 180_000_000L);
        for (int ms = 200; ms <= 380; ms += 20) frame(ms, false);
        assertFalse(detector.tipped());
        frame(400, false);
        assertTrue(detector.tipped());
    }

    @Test
    public void invalidTimestampCannotArmOrConfirmTip() {
        frame(0, true);
        detector.observe(0, 10, Double.NaN, false, START + 20_000_000L);
        detector.observe(0, 10, 0, false, START + 200_000_000L);
        assertEquals(HiveTipDetector.State.UNKNOWN, detector.state());
        assertFalse(detector.tipped());
    }

    @Test
    public void confirmedLossStaysLatchedForThisShot() {
        frame(0, true);
        for (int ms = 20; ms <= 220; ms += 20) frame(ms, false);
        frame(240, true);
        assertTrue(detector.tipped());
    }
}
