package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.robot.Mechanism;

public class Limelight implements Mechanism {
    public Limelight(){}

    private Limelight3A limelight;

    public void initialize(HardwareMap hardwareMap) {
        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        // A missing camera or a different device type under this name returns null.
        if (limelight == null) return;
        limelight.setPollRateHz(50);
        limelight.start();

    }


    public LLResult getLatestResult() {
        return limelight == null ? null : limelight.getLatestResult();
    }

    public boolean isAvailable() {
        return limelight != null;
    }

}
