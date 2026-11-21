package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

public final class HiveTipDetector {
    public enum State { WAITING_FOR_TAG, VISIBLE, TAG_LOST, UNKNOWN }
    private static final long LOSS_NANOS = 200_000_000L;
    private final int tagId;
    private final int pipeline;
    private double lastTimestamp = Double.NEGATIVE_INFINITY;
    private long lastFreshFrameNanos = Long.MIN_VALUE;
    private long missingSinceNanos = Long.MIN_VALUE;
    private State state = State.WAITING_FOR_TAG;

    public HiveTipDetector(int tagId, int pipeline) {
        this.tagId = tagId;
        this.pipeline = pipeline;
    }

    public void update(LLResult result, long now) {
        if (result == null) {
            unavailable(now);
            return;
        }
        boolean visible = false;
        if (result.isValid() && result.getFiducialResults() != null) {
            for (FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == tagId) {
                    visible = true;
                    break;
                }
            }
        }
        observe(result.getPipelineIndex(),
                result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency(),
                result.getTimestamp(), visible, now);
    }

    /** A fresh pipeline frame can be valid camera data even when it contains no targets. */
    void observe(int framePipeline, double ageMs, double timestamp, boolean visible, long now) {
        if (terminal() || cameraGap(now)) return;
        if (framePipeline != pipeline) {
            invalidate();
            return;
        }
        if (!(ageMs >= 0 && ageMs <= TurretConfig.maxFrameAgeMs)
                || !Double.isFinite(timestamp) || timestamp <= 0) {
            unavailable(now);
            return;
        }
        if (timestamp < lastTimestamp) {
            invalidate(); // Camera restarted or its frame clock changed during this shot.
            return;
        }
        if (timestamp == lastTimestamp) return;
        lastTimestamp = timestamp;
        lastFreshFrameNanos = now;
        if (visible) {
            state = State.VISIBLE;
            missingSinceNanos = Long.MIN_VALUE;
        } else if (state == State.VISIBLE) {
            if (missingSinceNanos == Long.MIN_VALUE) missingSinceNanos = now;
            else if (now - missingSinceNanos >= LOSS_NANOS) state = State.TAG_LOST;
        }
    }

    private void unavailable(long now) {
        if (terminal()) return;
        missingSinceNanos = Long.MIN_VALUE;
        cameraGap(now);
    }

    private boolean cameraGap(long now) {
        if (lastFreshFrameNanos != Long.MIN_VALUE && now - lastFreshFrameNanos >= LOSS_NANOS) {
            invalidate();
            return true;
        }
        return false;
    }

    private boolean terminal() {
        return state == State.UNKNOWN || state == State.TAG_LOST;
    }

    public void invalidate() { state = State.UNKNOWN; }
    public boolean isArmed() { return state == State.VISIBLE; }
    public boolean tipped() { return state == State.TAG_LOST; }
    public State state() { return state; }
}
