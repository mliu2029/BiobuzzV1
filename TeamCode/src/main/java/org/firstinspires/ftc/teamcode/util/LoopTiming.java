package org.firstinspires.ftc.teamcode.util;

import dev.nextftc.robot.opmode.OpModeHook;
import java.util.Arrays;
import java.util.Locale;

/** Start-to-start loop intervals, including the scheduler, motor updates, and telemetry. */
public final class LoopTiming implements OpModeHook {
    private final double[] samples = new double[256];
    private final double[] sorted = new double[256];
    private long lastNanos;
    private int count;
    private int next;

    @Override
    public void beforeStart() {
        lastNanos = 0;
        count = 0;
        next = 0;
    }

    @Override
    public void beforePeriodic() { record(System.nanoTime()); }

    public void record(long nowNanos) {
        if (lastNanos != 0) {
            samples[next] = (nowNanos - lastNanos) / 1e6;
            next = (next + 1) % samples.length;
            count = Math.min(count + 1, samples.length);
        }
        lastNanos = nowNanos;
    }

    /** Called only when telemetry is due; sampling itself makes no allocations. */
    public String summary() {
        if (count == 0) return "Waiting for loops";
        double total = 0;
        for (int i = 0; i < count; i++) {
            sorted[i] = samples[i];
            total += samples[i];
        }
        Arrays.sort(sorted, 0, count);
        return String.format(Locale.US, "%.1f / %.1f / %.1f",
                total / count, sorted[(int) Math.ceil(count * 0.95) - 1], sorted[count - 1]);
    }
}
