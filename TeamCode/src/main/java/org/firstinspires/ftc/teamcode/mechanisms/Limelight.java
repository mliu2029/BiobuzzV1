package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;

public class Limelight implements Mechanism {
    public Limelight() { }

    private Limelight3A camera;
    private int appliedPipeline = -1;

    public void initialize(HardwareMap hardwareMap) {
        camera = hardwareMap.tryGet(Limelight3A.class, "limelight");
        appliedPipeline = -1;
        if (camera != null) {
            camera.setPollRateHz(50);
            camera.start();
        }
    }

    public void selectPipeline(int pipeline) {
        if (camera != null && appliedPipeline != pipeline) {
            camera.pipelineSwitch(pipeline);
            appliedPipeline = pipeline;
        }
    }

    public LLResult getLatestResult() {
        return camera == null ? null : camera.getLatestResult();
    }

    public void stopImmediately() {
        if (camera != null) camera.stop();
    }
}
