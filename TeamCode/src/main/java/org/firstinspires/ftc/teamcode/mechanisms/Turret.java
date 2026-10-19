package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import java.util.Objects;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.aiming.TurretCalculations.HiveTarget;
import org.firstinspires.ftc.teamcode.aiming.TurretCalculations.Alliance;
import org.firstinspires.ftc.teamcode.aiming.TurretCalculations;
import org.firstinspires.ftc.teamcode.aiming.TurretConfig;

public class Turret implements Mechanism {
    private NextMotor motor;
    private DcMotorImplEx encoder;
    private Limelight3A camera;
    private Supplier<Pose> poseSupplier;
    private final PIDCoefficients gains = new PIDCoefficients(0, 0, 0);
    private final PIDController pid = new PIDController(gains);
    private HiveTarget target = HiveTarget.RED_CLOSE;
    private Alliance alliance;
    private boolean automaticTarget;
    private boolean enabled;
    private double targetDegrees = Double.NaN;
    private double visionTimestamp = Double.NaN;
    private double visionDegrees = Double.NaN;
    private int appliedPipeline = -1;
    private String mode = "OFF";

    /** Initialize with the turret physically forward; reset its encoder to zero. */
    public void initialize(HardwareMap hardwareMap, Supplier<Pose> poseSupplier, Alliance alliance) {
        this.alliance = Objects.requireNonNull(alliance, "Alliance is required");
        encoder = hardwareMap.get(DcMotorImplEx.class, "turret");
        encoder.setPower(0);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor = new NextMotor(() -> encoder);
        motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        camera = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (camera != null) {
            camera.setPollRateHz(50);
            camera.start();
        }
        this.poseSupplier = poseSupplier;
        target = alliance.close;
        stopImmediately();
    }

    @Override
    public Command getDefaultCommand() {
        return infinite(this::update).setEnd(condition -> stopImmediately());
    }

    public Command aimAt(HiveTarget selected) {
        return instant(() -> {
            if (selected == null || !alliance.correctTarget(selected)) {
                stopImmediately();
                return;
            }
            target = selected;
            automaticTarget = false;
            enabled = true;
            clearVision();
            pid.reset();
        });
    }


    public Command aimAutomatically() {
        return instant(() -> {
            automaticTarget = true;
            enabled = true;
            clearVision();
            pid.reset();
        });
    }

    public Command stop() {
        return instant(this::stopImmediately);
    }

    public void stopImmediately() {
        enabled = false;
        mode = "OFF";
        targetDegrees = Double.NaN;
        clearVision();
        pid.reset();
        applyPower(0);
    }

    private void clearVision() {
        visionTimestamp = Double.NaN;
        visionDegrees = Double.NaN;
    }

    /** Plain SDK autonomous OpModes must call this after follower.update() each loop. */
    public void update() {
        if (motor == null) return;
        targetDegrees = Double.NaN;
        if (!enabled || !calibrated()) {
            mode = !enabled ? "OFF" : "CALIBRATION REQUIRED";
            pid.reset();
            applyPower(0);
            return;
        }
        Pose pose = poseSupplier.get();
        if (pose == null) { stopImmediately(); return; }
        if (automaticTarget) {
            HiveTarget selected = alliance.targetForY(pose.y(), TurretConfig.halfwayY);
            if (selected == null) { stopImmediately(); return; }
            if (selected != target) {
                target = selected;
                clearVision();
                pid.reset();
            }
        }
        double current = getDegrees();
        double bearing = TurretCalculations.bearing(pose.x(), pose.y(), pose.heading(),
                TurretConfig.turretOffsetForwardInches, TurretConfig.turretOffsetLeftInches, target);
        targetDegrees = TurretCalculations.reachableAngle(bearing, current,
                TurretConfig.minDegrees, TurretConfig.maxDegrees);
        mode = "ODOMETRY";
        if (!Double.isFinite(targetDegrees)) {
            mode = "TARGET OUT OF RANGE / INVALID POSE";
            clearVision();
            pid.reset();
            applyPower(0);
            return;
        }
        updateVision(current);
        double output = calculatePower(targetDegrees - current);
        if ((current <= TurretConfig.minDegrees && output * TurretConfig.ticksPerDegree < 0)
                || (current >= TurretConfig.maxDegrees && output * TurretConfig.ticksPerDegree > 0)) {
            output = 0;
        }
        applyPower(output);
    }

    /** Keep correcting small errors too; tolerance only determines isAligned(). */
    double calculatePower(double error) {
        gains.kP = TurretConfig.p;
        gains.kI = TurretConfig.i;
        gains.kD = TurretConfig.d;
        double output = pid.calculate(error);
        output += Math.signum(error) * TurretConfig.staticPower;
        // Output sign follows encoder counts, while angles are always CCW-positive.
        output *= Math.signum(TurretConfig.ticksPerDegree);
        return Math.max(-TurretConfig.maxPower, Math.min(TurretConfig.maxPower, output));
    }

    private void updateVision(double current) {
        if (camera == null || !TurretConfig.useVision) {
            clearVision();
            return;
        }
        if (appliedPipeline != TurretConfig.pipeline) {
            camera.pipelineSwitch(TurretConfig.pipeline);
            appliedPipeline = TurretConfig.pipeline;
            clearVision();
        }
        LLResult result = camera.getLatestResult();
        if (result != null && result.isValid() && result.getFiducialResults() != null) {
            double sumError = 0;
            int accepted = 0;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (!target.acceptsTag(id)) continue;
                double txError = tag.getTargetXDegrees() - TurretConfig.desiredTx(target, id);
                double age = result.getStaleness() + result.getCaptureLatency()
                        + result.getTargetingLatency();
                if (!TurretCalculations.usableVision(target, id,
                        result.getPipelineIndex(), TurretConfig.pipeline, age,
                        TurretConfig.maxFrameAgeMs, txError, TurretConfig.acquireDegrees)) continue;
                sumError += txError;
                accepted++;
            }
            if (accepted > 0) {
                if (result.getTimestamp() != visionTimestamp) {
                    visionTimestamp = result.getTimestamp();
                    visionDegrees = TurretCalculations.reachableAngle(
                            current + TurretConfig.txToCcwSign * sumError / accepted, current,
                            TurretConfig.minDegrees, TurretConfig.maxDegrees);
                }
                // Repeated reads of one frame hold its setpoint; they do not add tx again.
                if (Double.isFinite(visionDegrees)) {
                    targetDegrees = visionDegrees;
                    mode = "LIMELIGHT";
                    return;
                }
            }
        }
        clearVision(); // Missing/wrong/stale tag immediately falls back to odometry.
    }

    private boolean calibrated() {
        double[] values = {TurretConfig.ticksPerDegree, TurretConfig.minDegrees,
                TurretConfig.maxDegrees, TurretConfig.p, TurretConfig.i, TurretConfig.d,
                TurretConfig.staticPower, TurretConfig.maxPower, TurretConfig.toleranceDegrees,
                TurretConfig.turretOffsetForwardInches, TurretConfig.turretOffsetLeftInches};
        for (double value : values) if (!Double.isFinite(value)) return false;
        return TurretConfig.ticksPerDegree != 0 && TurretConfig.minDegrees < 0
                && TurretConfig.maxDegrees > 0 && TurretConfig.p >= 0
                && TurretConfig.i >= 0 && TurretConfig.d >= 0 && TurretConfig.staticPower >= 0
                && TurretConfig.maxPower > 0 && TurretConfig.maxPower <= 1
                && TurretConfig.toleranceDegrees >= 0;
    }

    private void applyPower(double power) {
        if (motor == null) return;
        motor.setThrottle(Double.isFinite(power) ? power : 0);
        motor.update();
    }

    public double getDegrees() {
        return encoder == null || TurretConfig.ticksPerDegree == 0 ? Double.NaN
                : encoder.getCurrentPosition() / TurretConfig.ticksPerDegree;
    }

    public double getTargetDegrees() { return targetDegrees; }
    public int getEncoderTicks() { return encoder == null ? 0 : encoder.getCurrentPosition(); }
    public String getMode() { return mode; }
    public HiveTarget getTarget() { return target; }
    public boolean isAligned() {
        return enabled && Double.isFinite(targetDegrees)
                && Math.abs(targetDegrees - getDegrees()) <= TurretConfig.toleranceDegrees;
    }

    public void shutdown() {
        stopImmediately();
        if (camera != null) camera.stop();
    }
}
