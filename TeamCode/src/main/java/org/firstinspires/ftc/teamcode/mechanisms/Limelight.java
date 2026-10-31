package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;

public class Limelight implements Mechanism {
    public Limelight(){}

    private Limelight3A limelight;
    private int pipeline = -1;

    public void initialize(HardwareMap hardwareMap) {
        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        pipeline = -1;
        if (limelight == null) return;
        limelight.setPollRateHz(50);
        limelight.start();

    }

    /** Switches the camera pipeline; only sends a request when the index changes. */
    public void setPipeline(int index) {
        if (limelight == null || index == pipeline) return;
        if (limelight.pipelineSwitch(index)) pipeline = index;
    }


    public LLResult getLatestResult() {
        return limelight == null ? null : limelight.getLatestResult();
    }

    public boolean isAvailable() {
        return limelight != null;
    }

}
