package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveAimGeometry;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.util.TurretAngleEstimator;
import org.firstinspires.ftc.teamcode.util.TurretConfig;

public class Turret implements Mechanism {
    private enum Mode { OFF, MANUAL, LIMELIGHT, ODOMETRY }

    private final NextMotor turret = new NextMotor(RobotController.expansionHub(), 3,
            Units.Degrees.of(1.0 / TurretConfig.ticksPerDegree), 0.001);
    private final Limelight limelight;
    private final TurretAngleEstimator angleEstimator = new TurretAngleEstimator();
    private Supplier<Pose> poseSupplier;
    private Supplier<Velocity> velocitySupplier;
    private Alliance alliance;
    private HiveTarget target;
    private Mode mode = Mode.OFF;
    private boolean initialized;
    private boolean visionEnabled;
    private double startingDegrees;
    private double zeroDegrees;
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
        zeroDegrees = 0;
        poseSupplier = null;
        velocitySupplier = null;
        visionEnabled = false;
        recenterCount = 0;
        DcMotorController controller = RobotController.expansionHub().getMotorController();
        controller.setMotorPower(3, 0);
        controller.setMotorMode(3, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controller.setMotorMode(3, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setAnglePerCount(Units.Degrees.of(1.0 / TurretConfig.ticksPerDegree));
        turret.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        initialized = true;
        limelight.initialize(hardwareMap);
        resetOdometryAimCorrection();
        stopImmediately();
    }

    public void useOdometryAim(Supplier<Pose> pose, Supplier<Velocity> velocity) {
        poseSupplier = pose;
        velocitySupplier = velocity;
        resetOdometryAimCorrection();
        resumeAutoAim();
    }

    public void setAlliance(Alliance alliance) {
        boolean far = this.alliance != null && target == this.alliance.far;
        this.alliance = alliance;
        target = far ? alliance.far : alliance.close;
        resetOdometryAimCorrection();
    }

    public void setOdometryVisionEnabled(boolean enabled) {
        if (visionEnabled == enabled) return;
        visionEnabled = enabled;
        resetAim();
    }

    public void resetOdometryAimCorrection() {
        angleEstimator.reset();
        resetAim();
    }

    public void resumeAutoAim() {
        if (!initialized) return;
        if (mode == Mode.MANUAL) {
            zeroDegrees = turret.getEncoderPosition().into(Units.Degrees);
            startingDegrees = 0;
            recenterCount++;
            angleEstimator.reset();
        }
        manualDirection = 0;
        mode = poseSupplier == null ? Mode.LIMELIGHT : Mode.ODOMETRY;
        angleEstimator.resetStillness();
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
        if (!initialized || mode == Mode.OFF) return;
        turret.setAnglePerCount(Units.Degrees.of(1.0 / TurretConfig.ticksPerDegree));
        double current = getDegrees();
        if (mode == Mode.MANUAL) {
            double direction = manualDirection;
            if ((current <= TurretConfig.minDegrees && direction < 0)
                    || (current >= TurretConfig.maxDegrees && direction > 0)) direction = 0;
            turret.setThrottle(direction * TurretConfig.manualPower * Math.signum(TurretConfig.ticksPerDegree));
            motionStatus = direction == 0 ? "Manual: stopped" : "Manual bumper control";
            return;
        }

        int previousTag = visionTagId;
        visionTagId = -1;
        visionErrorDegrees = Double.NaN;
        visionFrameAgeMs = Double.NaN;
        visionPipeline = -1;
        visionStatus = "Vision disabled";
        targetDegrees = Double.NaN;
        Pose pose = poseSupplier == null ? null : poseSupplier.get();
        mode = poseSupplier == null ? Mode.LIMELIGHT : Mode.ODOMETRY;
        if (pose != null) updateOdometry(pose, current);
        if (visionEnabled || poseSupplier == null) updateVision(pose, current);
        if (visionTagId < 0) {
            trackedTagId = -1;
            visionDegrees = Double.NaN;
            angleEstimator.tagLost();
        }
        if (previousTag != visionTagId) turret.getPositionPID().reset();
        if (!Double.isFinite(targetDegrees)) {
            turret.setThrottle(0);
            turret.getPositionPID().reset();
            motionStatus = "No reachable target";
            return;
        }

        double sign = Math.signum(TurretConfig.ticksPerDegree);
        turret.getPositionConstants()
                .withP(sign * (mode == Mode.LIMELIGHT ? TurretConfig.visionP : TurretConfig.p))
                .withI(sign * TurretConfig.i)
                .withD(sign * TurretConfig.d)
                .withS(sign * TurretConfig.staticPower * Math.min(1,
                        Math.abs(targetDegrees - current) / TurretConfig.staticPowerRampDegrees));
        turret.setPositionSetpoint(Units.Degrees.of(targetDegrees - startingDegrees + zeroDegrees));
        motionStatus = isAligned() ? "Aligned; PID holding" : "Aiming";
    }

    private void updateOdometry(Pose pose, double current) {
        HiveTarget selected = alliance.targetForY(pose.y(), TurretConfig.halfwayY);
        if (selected == null) return;
        if (selected != target) {
            target = selected;
            trackedTagId = -1;
            turret.getPositionPID().reset();
        }
        Velocity velocity = velocitySupplier == null ? null : velocitySupplier.get();
        if (velocity != null) {
            angleEstimator.updateStillness(System.nanoTime(), Math.hypot(velocity.vx, velocity.vy),
                    Math.toDegrees(velocity.omega), getVelocityDegreesPerSecond());
        }
        targetDegrees = reachable(bearing(pose) - angleEstimator.getOffsetDegrees(), current);
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
        HiveTarget bestCell = null;
        double bestError = Double.NaN;
        double bestSetpoint = Double.NaN;
        for (FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            HiveTarget cell = alliance.close.acceptsTag(id) ? alliance.close
                    : alliance.far.acceptsTag(id) ? alliance.far : null;
            if (cell == null || (pose != null && cell != target)) continue;
            double error = cameraError(cell, tag, pose);
            if (!Double.isFinite(error) || Math.abs(error) >= TurretConfig.acquireDegrees) continue;
            double requested = sameFrame && id == trackedTagId ? visionDegrees
                    : current + (Math.abs(error) > TurretConfig.visionUpdateDegrees
                            ? TurretConfig.txToCcwSign * error : 0);
            double setpoint = reachable(requested, current);
            if (!Double.isFinite(setpoint)) continue;
            if (best == null || id == trackedTagId
                    || (best.getFiducialId() != trackedTagId && tag.getTargetArea() > best.getTargetArea())) {
                best = tag;
                bestCell = cell;
                bestError = error;
                bestSetpoint = setpoint;
            }
        }
        if (best == null) {
            visionStatus = "No usable tag for selected hive";
            return;
        }
        trackedTagId = visionTagId = best.getFiducialId();
        visionErrorDegrees = bestError;
        visionDegrees = targetDegrees = bestSetpoint;
        frameTimestamp = result.getTimestamp();
        frameReceiptTimestamp = result.getControlHubTimeStamp();
        target = bestCell;
        mode = Mode.LIMELIGHT;
        visionStatus = "2D opening: tag " + visionTagId;
        if (pose != null) {
            angleEstimator.observe(System.nanoTime(), visionFrameAgeMs, bearing(pose), current,
                    TurretConfig.txToCcwSign * bestError);
        }
    }

    private double cameraError(HiveTarget cell, FiducialResult tag, Pose pose) {
        return tag.getTargetXDegrees() + Math.toDegrees(Math.atan2(
                -HiveAimGeometry.tagOffsetInches(cell, tag.getFiducialId()),
                HiveAimGeometry.estimateDistanceInches(cell, pose)));
    }

    private double bearing(Pose pose) {
        return Calculations.bearing(pose.x(), pose.y(), pose.heading(), target);
    }

    private double reachable(double degrees, double current) {
        return Calculations.reachableAngle(degrees, current, TurretConfig.minDegrees, TurretConfig.maxDegrees);
    }

    public double getDegrees() {
        return initialized ? startingDegrees + turret.getEncoderPosition().into(Units.Degrees) - zeroDegrees
                : Double.NaN;
    }

    public double getVelocityDegreesPerSecond() {
        return turret.getEncoderVelocity().into(Units.DegreesPerSecond);
    }

    public int getEncoderTicks() {
        return (int) Math.round(turret.getEncoderPosition().into(Units.Degrees) * TurretConfig.ticksPerDegree);
    }

    public boolean isAligned() {
        return mode != Mode.OFF && mode != Mode.MANUAL
                && Math.abs(targetDegrees - getDegrees()) <= TurretConfig.toleranceDegrees;
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
    public double getChassisDegrees() { return angleEstimator.chassisDegrees(getDegrees()); }
    public double getVisionChassisDegrees() { return angleEstimator.getVisionChassisDegrees(); }
    public double getOdometryAngleOffsetDegrees() { return angleEstimator.getOffsetDegrees(); }
    public String getOdometryCorrectionStatus() { return visionEnabled ? angleEstimator.getStatus() : "Vision disabled"; }
}
