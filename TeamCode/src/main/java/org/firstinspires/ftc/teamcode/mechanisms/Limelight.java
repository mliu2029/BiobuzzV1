package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.control.geometry.Pose2d;
import dev.nextftc.hardware.webcams.NextLimelight;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.Distance;

public class Limelight implements Mechanism {
    private final NextLimelight limelight;
    private int pipeline = -1;

    public final Command startLimelight;
    public final Command stopLimelight;

    public Limelight() {
        this("limelight");
    }

    public Limelight(String name) {
        this.limelight = new NextLimelight(name);
        this.startLimelight = instant(() -> {
            try { limelight.start(); } catch (Exception ignored) {}
        });
        this.stopLimelight = instant(() -> {
            try { limelight.stop(); } catch (Exception ignored) {}
        });
    }

    public Command on() {
        return startLimelight;
    }

    public Command off() {
        return stopLimelight;
    }

    public Command setPipelineCommand(int index) {
        return instant(() -> setPipeline(index));
    }

    public void initialize(HardwareMap hardwareMap) {
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

//    public Distance distance(){
//        return limelight.getDistance();
//    }
//    public Pose2d pose(){
//
//    }
    public boolean isAvailable() {
        try {
            return limelight.isConnected();
        } catch (Exception e) {
            return false;
        }
    }

    public NextLimelight getNextLimelight() {
        return limelight;
    }
}
