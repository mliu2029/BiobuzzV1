package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

/** Observe the watched tag once, then count 200 ms without a fresh sighting as a tip. */
public final class HiveTipDetector {
    public enum State { WAITING_FOR_TAG, VISIBLE, TAG_LOST, UNKNOWN }
    private static final long LOSS_NANOS = 200_000_000L;
    private final int tagId;
    private final int pipeline;
    private double lastTimestamp = Double.NEGATIVE_INFINITY;
    private long lastSeenNanos;
    private State state = State.WAITING_FOR_TAG;

    public HiveTipDetector(int tagId, int pipeline) {
        this.tagId = tagId;
        this.pipeline = pipeline;
    }

    public void update(LLResult result, long now) {
        if (state == State.UNKNOWN || state == State.TAG_LOST) return;
        if (result != null && result.getPipelineIndex() == pipeline) {
            double age = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
            double timestamp = result.getTimestamp();
            if (age >= 0 && age <= TurretConfig.maxFrameAgeMs
                    && Double.isFinite(timestamp) && timestamp > 0 && timestamp > lastTimestamp) {
                lastTimestamp = timestamp;
                if (result.isValid() && result.getFiducialResults() != null) {
                    for (FiducialResult tag : result.getFiducialResults()) {
                        if (tag.getFiducialId() != tagId) continue;
                        lastSeenNanos = now;
                        state = State.VISIBLE;
                        return;
                    }
                }
            }
        }
        if (state == State.VISIBLE && now - lastSeenNanos >= LOSS_NANOS) state = State.TAG_LOST;
    }

    public void invalidate() { state = State.UNKNOWN; }
    public boolean isArmed() { return state == State.VISIBLE; }
    public boolean tipped() { return state == State.TAG_LOST; }
    public State state() { return state; }
}
