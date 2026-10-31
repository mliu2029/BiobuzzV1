package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import java.util.Objects;
import org.firstinspires.ftc.teamcode.data.HiveTarget;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.Calculations;
import org.firstinspires.ftc.teamcode.data.TurretConfig;

public class Turret implements Mechanism {
    private final Limelight limelight;

    public Turret(Limelight limelight) {
        this.limelight = Objects.requireNonNull(limelight);
    }
    private NextMotor turret;
    private DcMotorImplEx turretMotor;
    private final PIDCoefficients gains = new PIDCoefficients(0.01, 0, 0.001);
    private final PIDController pid = new PIDController(gains);
    private HiveTarget target = HiveTarget.RED_CLOSE;
    private Alliance alliance;
    private boolean enabled;
    private boolean manualControl;
    private double manualDirection;
    private double startingDegrees;
    private int encoderZeroTicks;
    private double targetDegrees = Double.NaN;
    private double fixedDegrees = Double.NaN;
    private double visionTimestamp = Double.NaN;
    private double visionDegrees = Double.NaN;
    private int visionTagId = -1;
    private String mode = "OFF";
    private String visionStatus = "Auto aim off";
    private double visionFrameAgeMs = Double.NaN;
    private int visionPipeline = -1;
    private String visionTags = "[]";
    private double visionNearestErrorDegrees = Double.NaN;

    public final Command aimTurret = instant(this::resumeAutoAim);
    public final Command stopTurret = instant(this::stopImmediately);

    /** Resume Limelight aiming, zeroing the turret first when confirming a manual recenter. */
    public void resumeAutoAim() {
        applyPower(0);
        if (manualControl) {
            // Rebase the angle without resetting hardware while bulk reads are cached.
            encoderZeroTicks = turretMotor.getCurrentPosition();
            startingDegrees = 0;
        }
        manualControl = false;
        manualDirection = 0;
        fixedDegrees = Double.NaN;
        targetDegrees = Double.NaN;
        enabled = true;
        mode = "LIMELIGHT ONLY: WAITING";
        clearVision();
        resetVisionDiagnostics("Awaiting auto aim update");
        pid.reset();
    }

    /** Enter bumper control, or mark the physically recentered position as zero and resume aiming. */
    public void toggleManualRecenter() {
        if (turretMotor == null) return;
        if (manualControl) {
            resumeAutoAim();
        } else {
            stopImmediately();
            manualControl = true;
            mode = "MANUAL RECENTER";
            resetVisionDiagnostics("Manual recenter; auto aim off");
        }
    }

    /** Positive direction turns counterclockwise; both or neither bumper stops motion. */
    public void setManualInput(boolean leftBumper, boolean rightBumper) {
        manualDirection = manualControl && leftBumper != rightBumper
                ? (leftBumper ? 1 : -1) : 0;
    }

    /**
     * Moves to a robot-relative angle (positive is counterclockwise), then holds it.
     * Waits until within toleranceDegrees (or cancelled) so an autonomous sequence can continue.
     * Uses getDegrees()'s reference, including the starting angle passed to initialize().
     * aimTurret resumes automatic aiming.
     * Requires initialized hardware and calibrated limits; invalid targets are rejected.
     */
    public Command moveToDegrees(double degrees) {
        if (!Double.isFinite(degrees)) {
            throw new IllegalArgumentException("Turret angle must be finite");
        }
        return Command.build()
                .requiring(this)
                .setStart(() -> {
                    stopImmediately();
                    if (turret == null || !calibrated()) {
                        throw new IllegalStateException("Initialize and calibrate the turret before moving it");
                    }
                    if (degrees < TurretConfig.minDegrees || degrees > TurretConfig.maxDegrees) {
                        throw new IllegalArgumentException("Turret angle is outside the configured limits: " + degrees);
                    }
                    fixedDegrees = degrees;
                    targetDegrees = degrees;
                    mode = "POSITION";
                    enabled = true;
                })
                .setDone(this::isAligned)
                .setEnd(condition -> {
                    if (condition != EndCondition.NATURALLY) stopImmediately();
                });
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance) {
        initialize(hardwareMap, alliance, 0.0);
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance, double startingDegrees) {
        this.alliance = Objects.requireNonNull(alliance, "Alliance is required");
        this.startingDegrees = Double.isFinite(startingDegrees) ? startingDegrees : 0.0;
        encoderZeroTicks = 0;
        turretMotor = hardwareMap.get(DcMotorImplEx.class, "turret");
        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (turret == null) turret = new NextMotor(() -> turretMotor);
        turret.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        limelight.initialize(hardwareMap);
        target = alliance.close;
        stopImmediately();
    }

    public void stopImmediately() {
        enabled = false;
        manualControl = false;
        manualDirection = 0;
        mode = "OFF";
        targetDegrees = Double.NaN;
        fixedDegrees = Double.NaN;
        clearVision();
        resetVisionDiagnostics("Auto aim off");
        pid.reset();
        applyPower(0);
    }

    private void clearVision() {
        visionTimestamp = Double.NaN;
        visionDegrees = Double.NaN;
        visionTagId = -1;
    }

    private void resetVisionDiagnostics(String status) {
        visionStatus = status;
        visionFrameAgeMs = Double.NaN;
        visionPipeline = -1;
        visionTags = "[]";
        visionNearestErrorDegrees = Double.NaN;
    }

    @Override
    public void periodic() {
        if (turret == null) return;
        resetVisionDiagnostics("Not evaluated");
        targetDegrees = Double.NaN;
        if (manualControl) {
            mode = "MANUAL RECENTER";
            visionStatus = "Manual recenter; auto aim off";
            clearVision();
            pid.reset();
            double current = getDegrees();
            double power = calibrated() && Double.isFinite(TurretConfig.manualPower)
                    ? manualDirection * Math.max(0, Math.min(TurretConfig.maxPower,
                            TurretConfig.manualPower)) * Math.signum(TurretConfig.ticksPerDegree)
                    : 0;
            if (!Double.isFinite(current)
                    || (current <= TurretConfig.minDegrees && power * TurretConfig.ticksPerDegree < 0)
                    || (current >= TurretConfig.maxDegrees && power * TurretConfig.ticksPerDegree > 0)) {
                power = 0;
            }
            applyPower(power);
            return;
        }
        if (!enabled || !calibrated()) {
            mode = !enabled ? "OFF" : "CALIBRATION REQUIRED";
            visionStatus = !enabled ? "Auto aim off" : "Turret calibration required";
            pid.reset();
            applyPower(0);
            return;
        }
        double current = getDegrees();
        if (Double.isFinite(fixedDegrees)) {
            // Fixed angles stay relative to the chassis, even as the robot drives.
            if (fixedDegrees >= TurretConfig.minDegrees && fixedDegrees <= TurretConfig.maxDegrees) {
                targetDegrees = fixedDegrees;
            }
            mode = "POSITION";
            visionStatus = "Fixed angle command; auto aim inactive";
        } else {
            mode = "LIMELIGHT ONLY: WAITING";
            updateVision(current);
        }
        if (!Double.isFinite(targetDegrees)) {
            clearVision();
            pid.reset();
            applyPower(0);
            return;
        }
        double output = calculatePower(targetDegrees - current);
        if ((current <= TurretConfig.minDegrees && output * TurretConfig.ticksPerDegree < 0)
                || (current >= TurretConfig.maxDegrees && output * TurretConfig.ticksPerDegree > 0)) {
            output = 0;
        }
        applyPower(output);
    }

    double calculatePower(double error) {
        gains.kP = TurretConfig.p;
        gains.kI = TurretConfig.i;
        gains.kD = TurretConfig.d;
        double output = pid.calculate(error);
        output += Math.signum(error) * TurretConfig.staticPower;
        output *= Math.signum(TurretConfig.ticksPerDegree);
        return Math.max(-TurretConfig.maxPower, Math.min(TurretConfig.maxPower, output));
//        counter clockwise is positive
    }

    private void updateVision(double current) {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            visionStatus = limelight.isAvailable() ? "No camera result"
                    : "Camera unavailable; check USB/config";
            clearVision();
            return;
        }

        double age = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        visionFrameAgeMs = age;
        visionPipeline = result.getPipelineIndex();
        StringBuilder tags = new StringBuilder("[");
        if (result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tags.length() > 1) tags.append(", ");
                tags.append(tag.getFiducialId());
            }
        }
        visionTags = tags.append(']').toString();
        if (!result.isValid() || result.getFiducialResults() == null) {
            visionStatus = !result.isValid() ? "Camera result invalid / no target"
                    : "No AprilTag results";
            clearVision();
            return;
        }
        double acquireDegrees = TurretConfig.acquireDegrees;
        selectVisibleHive(result, age, acquireDegrees);
        double firstError = Double.NaN;
        double secondError = Double.NaN;
        int matching = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (!target.acceptsTag(id)) continue;
            matching++;
            double txError = tag.getTargetXDegrees() - TurretConfig.desiredTx(target, id);
            if (Double.isFinite(txError)) {
                visionNearestErrorDegrees = Double.isNaN(visionNearestErrorDegrees)
                        ? Math.abs(txError) : Math.min(visionNearestErrorDegrees, Math.abs(txError));
            }
            if (!Calculations.usableVision(target, id,
                    result.getPipelineIndex(), TurretConfig.pipeline, age,
                    TurretConfig.maxFrameAgeMs, txError, acquireDegrees)) continue;
            if (id == target.firstAimTagId) firstError = txError;
            else secondError = txError;
        }
        int selectedId = Calculations.selectVisionTag(target, visionTagId,
                Double.isFinite(firstError), Double.isFinite(secondError));
        if (selectedId < 0) {
            if (visionPipeline != TurretConfig.pipeline) {
                visionStatus = "Pipeline mismatch; expected " + TurretConfig.pipeline;
            } else if (!Double.isFinite(age) || age < 0) {
                visionStatus = "Invalid frame age";
            } else if (age > TurretConfig.maxFrameAgeMs) {
                visionStatus = "Frame too old; max " + TurretConfig.maxFrameAgeMs + " ms";
            } else if (matching == 0) {
                visionStatus = "No middle tags for " + target + "; expected "
                        + target.firstAimTagId + " or " + target.secondAimTagId;
            } else {
                visionStatus = "Invalid tx/offset or outside acquisition window: "
                        + acquireDegrees + " deg";
            }
            clearVision(); // Without a usable camera target, periodic() stops the motor.
            return;
        }

        double selectedError = selectedId == target.firstAimTagId ? firstError : secondError;
        if (result.getTimestamp() != visionTimestamp || selectedId != visionTagId) {
            visionTimestamp = result.getTimestamp();
            visionDegrees = Calculations.reachableAngle(
                    current + TurretConfig.txToCcwSign * selectedError, current,
                    TurretConfig.minDegrees, TurretConfig.maxDegrees);
        }
        // Repeated reads of one frame hold its setpoint; they do not add tx again.
        if (Double.isFinite(visionDegrees)) {
            visionTagId = selectedId;
            targetDegrees = visionDegrees;
            mode = "LIMELIGHT ONLY";
            visionStatus = "Tracking tag " + visionTagId;
        } else {
            visionStatus = "Vision target outside turret limits / invalid correction";
            clearVision();
        }
    }

    private void selectVisibleHive(LLResult result, double age, double acquireDegrees) {
        boolean closeUsable = false;
        boolean farUsable = false;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            HiveTarget observed = alliance.close.acceptsTag(id) ? alliance.close
                    : alliance.far.acceptsTag(id) ? alliance.far : null;
            if (observed == null) continue;
            double error = tag.getTargetXDegrees() - TurretConfig.desiredTx(observed, id);
            if (!Calculations.usableVision(observed, id, result.getPipelineIndex(),
                    TurretConfig.pipeline, age, TurretConfig.maxFrameAgeMs,
                    error, acquireDegrees)) continue;
            if (observed == alliance.close) closeUsable = true;
            else farUsable = true;
        }
        // Retain the current hive while usable, just as we retain one tag within that hive.
        if ((target == alliance.close && closeUsable) || (target == alliance.far && farUsable)) return;
        HiveTarget selected = closeUsable ? alliance.close : farUsable ? alliance.far : null;
        if (selected != null && selected != target) {
            target = selected;
            clearVision();
            pid.reset();
        }
    }

    private boolean calibrated() {
        double[] values = {TurretConfig.ticksPerDegree, TurretConfig.minDegrees,
                TurretConfig.maxDegrees, TurretConfig.p, TurretConfig.i, TurretConfig.d,
                TurretConfig.staticPower, TurretConfig.maxPower, TurretConfig.toleranceDegrees};
        for (double value : values) if (!Double.isFinite(value)) return false;
        return TurretConfig.ticksPerDegree != 0 && TurretConfig.minDegrees < 0
                && TurretConfig.maxDegrees > 0 && TurretConfig.p >= 0
                && TurretConfig.i >= 0 && TurretConfig.d >= 0 && TurretConfig.staticPower >= 0
                && TurretConfig.maxPower > 0 && TurretConfig.maxPower <= 1
                && TurretConfig.toleranceDegrees >= 0;
    }

    private void applyPower(double power) {
        if (turret == null) return;
        turret.setThrottle(Double.isFinite(power) ? power : 0);
        turret.update();
    }

    public double getDegrees() {
        return turretMotor == null || TurretConfig.ticksPerDegree == 0 ? Double.NaN
                : startingDegrees + ((double) turretMotor.getCurrentPosition() - encoderZeroTicks)
                        / TurretConfig.ticksPerDegree;
    }

    public double getTargetDegrees() { return targetDegrees; }
    public int getEncoderTicks() { return turretMotor == null ? 0 : turretMotor.getCurrentPosition(); }
    public boolean isManualControl() { return manualControl; }
    public String getMode() { return mode; }
    public String getVisionStatus() { return visionStatus; }
    public double getVisionFrameAgeMs() { return visionFrameAgeMs; }
    public int getVisionPipeline() { return visionPipeline; }
    public String getVisionTags() { return visionTags; }
    public double getVisionNearestErrorDegrees() { return visionNearestErrorDegrees; }
    public HiveTarget getTarget() { return target; }
    public boolean isAligned() {
        return enabled && Double.isFinite(targetDegrees)
                && Math.abs(targetDegrees - getDegrees()) <= TurretConfig.toleranceDegrees;
    }

}
