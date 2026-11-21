package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.util.TurretConfig;

public class Turret implements Mechanism {
    private enum Mode { OFF, MANUAL, LIMELIGHT, ODOMETRY }
    private static final double[] TAG_X_INCHES = {-6.5, -2.75, 2.75, 6.5};

    private DcMotorImplEx turretMotor;
    // The named SDK motor supplies both raw REV quadrature ticks and NextMotor feedback.
    private final NextMotor turret = new NextMotor("turret", Units.Degrees.of(0), 0.001);
    private final Limelight limelight;
    private Supplier<Pose> poseSupplier;
    private Alliance alliance;
    private HiveTarget target;
    private Mode mode = Mode.OFF;
    private boolean initialized;
    private boolean visionEnabled;
    private double startingDegrees;
    private int zeroTicks;
    private double degreesPerEncoderTick = Double.NaN;
    private double manualDirection;
    private double targetDegrees = Double.NaN;
    private int recenterCount;

    private int visionTagId = -1;
    private double visionErrorDegrees = Double.NaN;
    private double visionFrameAgeMs = Double.NaN;
    private int visionPipeline = -1;
    private String visionStatus = "Vision disabled";
    private String motionStatus = "Auto aim off";
    private int trackedTagId = -1;
    private double visionDegrees = Double.NaN;
    private double frameTimestamp = Double.NaN;
    private long frameReceiptTimestamp = Long.MIN_VALUE;

    public final Command aimTurret = instant(this::resumeAutoAim);
    public final Command stopTurret = instant(this::stopImmediately);
    public final Command toggleRecenter = instant(this::toggleManualRecenter);
    public final Command manualLeft = instant(() -> setManualInput(true, false));
    public final Command manualRight = instant(() -> setManualInput(false, true));
    public final Command manualStop = instant(() -> setManualInput(false, false));

    public Turret(Limelight limelight) {
        this.limelight = limelight;
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance) {
        initialize(hardwareMap, alliance, 0);
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance, double startingDegrees) {
        this.alliance = alliance;
        target = alliance.close;
        this.startingDegrees = startingDegrees;
        zeroTicks = 0;
        poseSupplier = null;
        visionEnabled = false;
        recenterCount = 0;
        turretMotor = hardwareMap.get(DcMotorImplEx.class, "turret");
        turretMotor.setPower(0);
        // Relative encoder: the caller establishes the physical starting angle.
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        updateEncoderScale();
        turret.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        initialized = true;
        limelight.initialize(hardwareMap);
        resetAimTracking();
        stopImmediately();
    }

    public void useOdometryAim(Supplier<Pose> pose) {
        poseSupplier = pose;
        resetAimTracking();
        resumeAutoAim();
    }

    public void setAlliance(Alliance alliance) {
        boolean far = this.alliance != null && target == this.alliance.far;
        this.alliance = alliance;
        target = far ? alliance.far : alliance.close;
        resetAimTracking();
    }

    public void setOdometryVisionEnabled(boolean enabled) {
        if (visionEnabled == enabled) return;
        visionEnabled = enabled;
        resetAim();
    }

    /** Clear target tracking after a pose/mode change without changing the encoder zero. */
    public void resetAimTracking() {
        resetAim();
    }

    public void resumeAutoAim() {
        if (!initialized) return;
        if (mode == Mode.MANUAL) {
            zeroTicks = getEncoderTicks();
            startingDegrees = 0;
            recenterCount++;
        }
        manualDirection = 0;
        mode = Mode.ODOMETRY;
        resetAim();
    }

    public void toggleManualRecenter() {
        if (!initialized) return;
        if (mode == Mode.MANUAL) {
            resumeAutoAim();
        } else {
            stopImmediately();
            mode = Mode.MANUAL;
        }
    }

    public void setManualInput(boolean left, boolean right) {
        manualDirection = mode == Mode.MANUAL && left != right ? (left ? 1 : -1) : 0;
    }

    public void stopImmediately() {
        mode = Mode.OFF;
        manualDirection = 0;
        motionStatus = "Auto aim off";
        resetAim();
        if (initialized) turret.update();
    }

    private void resetAim() {
        targetDegrees = Double.NaN;
        visionTagId = -1;
        trackedTagId = -1;
        visionDegrees = Double.NaN;
        visionErrorDegrees = Double.NaN;
        turret.getPositionPID().reset();
        turret.setThrottle(0);
    }

    @Override
    public void periodic() {
        if (!initialized) return;
        updateEncoderScale();
        if (!TurretConfig.isEncoderConfigured()) {
            resetAim();
            motionStatus = "Turret disabled: configure encoder ratio and direction";
            turret.update();
            return;
        }
        if (mode == Mode.OFF) return;
        double current = getDegrees();
        if (mode == Mode.MANUAL) {
            double direction = manualDirection;
            if ((current <= TurretConfig.minDegrees && direction < 0)
                    || (current >= TurretConfig.maxDegrees && direction > 0)) direction = 0;
            turret.setThrottle(direction * TurretConfig.manualPower);
            motionStatus = direction == 0 ? "Manual: stopped" : "Manual bumper control";
            return;
        }

        visionTagId = -1;
        visionErrorDegrees = Double.NaN;
        visionFrameAgeMs = Double.NaN;
        visionPipeline = -1;
        visionStatus = "Vision disabled";
        targetDegrees = Double.NaN;
        Pose pose = poseSupplier == null ? null : poseSupplier.get();
        mode = Mode.ODOMETRY;
        if (pose != null) {
            // Odometry picks the hive and coarse angle; vision only refines that same target.
            updateOdometry(pose, current);
            if (visionEnabled && Double.isFinite(targetDegrees)) updateVision(pose, current);
        } else {
            visionStatus = "Waiting for odometry pose";
        }
        if (visionTagId < 0) {
            trackedTagId = -1;
            visionDegrees = Double.NaN;
        }
        if (!Double.isFinite(targetDegrees)) {
            turret.setThrottle(0);
            turret.getPositionPID().reset();
            motionStatus = "No reachable target";
            return;
        }

        turret.getPositionConstants()
                .withP(TurretConfig.p)
                .withI(TurretConfig.i)
                .withD(TurretConfig.d)
                .withS(TurretConfig.staticPower * Math.min(1,
                        Math.abs(targetDegrees - current) / TurretConfig.staticPowerRampDegrees));
        turret.setPositionSetpoint(Units.Degrees.of(
                targetDegrees - startingDegrees + zeroTicks * degreesPerEncoderTick));
        motionStatus = isAligned() ? "Aligned; PID holding" : "Aiming";
    }

    private void updateEncoderScale() {
        double scale = TurretConfig.isEncoderConfigured() ? TurretConfig.getDegreesPerEncoderTick() : 0;
        if (Double.compare(scale, degreesPerEncoderTick) == 0) return;
        degreesPerEncoderTick = scale;
        turret.setAnglePerCount(Units.Degrees.of(scale));
        resetAim();
    }

    private void updateOdometry(Pose pose, double current) {
        HiveTarget selected = alliance.targetForY(pose.y(), TurretConfig.halfwayY);
        if (selected == null) return;
        if (selected != target) {
            target = selected;
            trackedTagId = -1;
            visionDegrees = Double.NaN;
            turret.getPositionPID().reset();
        }
        targetDegrees = reachable(bearing(pose), current);
    }

    private void updateVision(Pose pose, double current) {
        limelight.setPipeline(TurretConfig.pipeline);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            visionStatus = "No camera target";
            return;
        }
        visionFrameAgeMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        visionPipeline = result.getPipelineIndex();
        if (visionPipeline != TurretConfig.pipeline
                || !(visionFrameAgeMs >= 0 && visionFrameAgeMs <= TurretConfig.maxFrameAgeMs)) {
            visionStatus = "Waiting for a fresh camera frame";
            return;
        }
        boolean sameFrame = result.getTimestamp() > 0
                ? result.getTimestamp() == frameTimestamp
                : result.getControlHubTimeStamp() == frameReceiptTimestamp;
        FiducialResult best = null;
        double bestError = Double.NaN;
        double bestSetpoint = Double.NaN;
        for (FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (!target.acceptsTag(id)) continue;
            double error = cameraError(target, tag, pose);
            if (!Double.isFinite(error) || Math.abs(error) >= TurretConfig.acquireDegrees) continue;
            double requested = sameFrame && id == trackedTagId ? visionDegrees
                    : current + (Math.abs(error) > TurretConfig.visionUpdateDegrees
                            ? TurretConfig.txToCcwSign * error : 0);
            double setpoint = reachable(requested, current);
            if (!Double.isFinite(setpoint)) continue;
            if (best == null || id == trackedTagId
                    || (best.getFiducialId() != trackedTagId && tag.getTargetArea() > best.getTargetArea())) {
                best = tag;
                bestError = error;
                bestSetpoint = setpoint;
            }
        }
        if (best == null) {
            visionStatus = "Odometry aim; no usable tag for " + target;
            return;
        }
        trackedTagId = visionTagId = best.getFiducialId();
        visionErrorDegrees = bestError;
        visionDegrees = targetDegrees = bestSetpoint;
        frameTimestamp = result.getTimestamp();
        frameReceiptTimestamp = result.getControlHubTimeStamp();
        mode = Mode.LIMELIGHT;
        visionStatus = "Limelight correction: tag " + visionTagId;
    }

    private double cameraError(HiveTarget cell, FiducialResult tag, Pose pose) {
        double distance = pose == null ? 50 : Math.hypot(cell.x - pose.x(), cell.y - pose.y());
        if (!Double.isFinite(distance) || distance < 12) distance = 50;
        double tagX = TAG_X_INCHES[tag.getFiducialId() - cell.firstTagId];
        return tag.getTargetXDegrees() + Math.toDegrees(Math.atan2(-tagX, distance))
                + TurretConfig.visionOffsetDegrees;
    }

    private double bearing(Pose pose) {
        return Calculations.bearing(pose.x(), pose.y(), pose.heading(), target);
    }

    private double reachable(double degrees, double current) {
        return Calculations.reachableAngle(degrees, current, TurretConfig.minDegrees, TurretConfig.maxDegrees);
    }

    /** Turret angle from REV quadrature counts and the explicit startup/manual zero only. */
    public double getDegrees() {
        return initialized && TurretConfig.isEncoderConfigured()
                ? startingDegrees + (getEncoderTicks() - (double) zeroTicks) * TurretConfig.getDegreesPerEncoderTick()
                : Double.NaN;
    }

    public double getVelocityDegreesPerSecond() {
        return initialized && TurretConfig.isEncoderConfigured()
                ? turretMotor.getVelocity() * TurretConfig.getDegreesPerEncoderTick() : Double.NaN;
    }

    public int getEncoderTicks() {
        return initialized ? turretMotor.getCurrentPosition() : 0;
    }

    public double getPidErrorDegrees() { return targetDegrees - getDegrees(); }

    public String getEncoderConfigurationStatus() {
        return TurretConfig.isEncoderConfigured()
                ? "REV V2 / ratio " + TurretConfig.encoderRevolutionsPerTurretRevolution
                        + " / direction " + TurretConfig.encoderDirection
                : "INVALID ratio/direction: turret output disabled";
    }

    public boolean isAligned() {
        return mode != Mode.OFF && mode != Mode.MANUAL
                && Math.abs(targetDegrees - getDegrees()) <= TurretConfig.toleranceDegrees
                && (mode != Mode.LIMELIGHT || isVisionAligned());
    }

    public boolean isVisionAligned() {
        return visionTagId >= 0 && Math.abs(visionErrorDegrees) <= TurretConfig.toleranceDegrees;
    }

    public HiveTarget getTarget() { return target; }
    public double getTargetDegrees() { return targetDegrees; }
    public double getPower() { return initialized ? Math.max(-1, Math.min(1, turret.getThrottle())) : 0; }
    public String getMode() { return mode.name(); }
    public String getMotionStatus() { return motionStatus; }
    public String getVisionStatus() {
        return mode == Mode.OFF ? "Auto aim off" : mode == Mode.MANUAL ? "Manual recenter" : visionStatus;
    }
    public int getVisionTagId() { return visionTagId; }
    public double getVisionErrorDegrees() { return visionErrorDegrees; }
    public double getVisionFrameAgeMs() { return visionFrameAgeMs; }
    public int getVisionPipeline() { return visionPipeline; }
    public int getRecenterCount() { return recenterCount; }
}
