package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;

import dev.nextftc.hardware.webcams.NextLimelight;
import dev.nextftc.robot.Mechanism;

public class Limelight implements Mechanism {
    private final NextLimelight limelight = new NextLimelight("limelight");
    private int pipeline = -1;

    public void initialize() {
        pipeline = -1;
        try {
            limelight.setPollRate(50);
            limelight.start();
        } catch (Exception ignored) {
        }
    }

    public void setPipeline(int index) {
        if (index == pipeline) return;
        try {
            if (limelight.setPipeline(index)) {
                pipeline = index;
            }
        } catch (Exception ignored) {
        }
    }

    public LLResult getLatestResult() {
        try {
            return limelight.getLatestResult();
        } catch (Exception e) {
            return null;
        }
    }

}
