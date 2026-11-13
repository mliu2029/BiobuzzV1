package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.util.HiveTarget;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Calculations;
import org.firstinspires.ftc.teamcode.util.HiveAimGeometry;
import org.firstinspires.ftc.teamcode.util.TurretConfig;
import org.firstinspires.ftc.teamcode.util.TurretAngleEstimator;

public class Turret implements Mechanism {
    private static final double POWER_CACHE_TOLERANCE = 0.001;
    private final Limelight limelight;
    private NextMotor turret;
    private DcMotorImplEx turretMotor;
    private final PIDCoefficients gains = new PIDCoefficients(0.01, 0, 0.001);
    private final PIDController pid = new PIDController(gains);
    private HiveTarget target = HiveTarget.RED_CLOSE;
    private Alliance alliance;
    private Supplier<Pose> odometryPoseSupplier;
    private Supplier<Velocity> odometryVelocitySupplier;
    private final TurretAngleEstimator angleEstimator = new TurretAngleEstimator();
    private long aimUpdateTimeNanos;
    private DoubleSupplier movingAimDegrees;
    private DoubleSupplier movingAimRate;
    private boolean odometryVisionEnabled;
    private boolean enabled;
    private boolean manualControl;
    private double manualDirection;
    private double startingDegrees;
    private int encoderZeroTicks;
    private double targetDegrees = Double.NaN;
    private double visionTimestamp = Double.NaN;
    private long visionReceiptTimestamp = Long.MIN_VALUE;
    private double visionDegrees = Double.NaN;
    private int visionTagId = -1;
    private boolean visionSettled;
    private String mode = "OFF";
    private String visionStatus = "Auto aim off";
    private double visionFrameAgeMs = Double.NaN;
    private int visionPipeline = -1;
    private String visionTags = "[]";
    private double visionErrorDegrees = Double.NaN;
    private double visionRequestedDegrees = Double.NaN;
    private int visionErrorTagId = -1;
    private String motionStatus = "Auto aim off";
    private double lastRecenterDegrees = Double.NaN;
    private int recenterCount;

    public final Command aimTurret = instant(this::resumeAutoAim);
    public final Command stopTurret = instant(this::stopImmediately);

    public Turret(Limelight limelight) {
        this.limelight = Objects.requireNonNull(limelight);
    }

    /** Resume the selected aiming source, zeroing after a manual recenter. */
    public void resumeAutoAim() {
        applyPower(0);
        if (manualControl) {
            // Rebase the angle without resetting hardware while bulk reads are cached.
            lastRecenterDegrees = getDegrees();
            recenterCount++;
            encoderZeroTicks = turretMotor.getCurrentPosition();
            startingDegrees = 0;
            resetOdometryAimCorrection();
        }
        manualControl = false;
        manualDirection = 0;
        targetDegrees = Double.NaN;
        enabled = true;
        mode = movingAimDegrees != null ? "SHOOTING ON THE MOVE"
                : odometryPoseSupplier == null ? "LIMELIGHT ONLY: WAITING" : "ODOMETRY";
        motionStatus = movingAimDegrees != null ? "Waiting for ballistic solution"
                : odometryPoseSupplier == null ? "Waiting for camera" : "Waiting for pose";
        clearVision();
        resetVisionDiagnostics("Awaiting auto aim update");
        angleEstimator.resetStillness();
        pid.reset();
    }

    public void setAlliance(Alliance alliance) {
        Objects.requireNonNull(alliance, "Alliance is required");
        if (this.alliance == alliance) return;
        boolean wasFar = this.alliance != null && target == this.alliance.far;
        this.alliance = alliance;
        target = wasFar ? alliance.far : alliance.close;
        angleEstimator.reset();
        resetAim();
    }

    public void useOdometryAim(Supplier<Pose> poseSupplier, Supplier<Velocity> velocitySupplier) {
        if (turretMotor == null) throw new IllegalStateException("Initialize the turret first");
        movingAimDegrees = null;
        movingAimRate = null;
        odometryPoseSupplier = Objects.requireNonNull(poseSupplier);
        odometryVelocitySupplier = Objects.requireNonNull(velocitySupplier);
        resetOdometryAimCorrection();
        resumeAutoAim();
    }

    /** Forget the camera-corrected reference when the chassis pose or turret zero is reset. */
    public void resetOdometryAimCorrection() {
        angleEstimator.reset();
        clearVision();
    }

    /** Continuously track a robot-relative ballistic setpoint and its angular velocity. */
    public void useMovingAim(DoubleSupplier angleDegrees, DoubleSupplier rateDegreesPerSecond) {
        if (turretMotor == null) throw new IllegalStateException("Initialize the turret first");
        movingAimDegrees = Objects.requireNonNull(angleDegrees);
        movingAimRate = Objects.requireNonNull(rateDegreesPerSecond);
        odometryPoseSupplier = null;
        odometryVelocitySupplier = null;
        resetOdometryAimCorrection();
        resumeAutoAim();
    }

    public boolean isTrackingMovingAim() {
        return enabled && !manualControl && movingAimDegrees != null
                && calibrated() && Double.isFinite(TurretConfig.movingVelocityFeedforward)
                && TurretConfig.movingVelocityFeedforward >= 0;
    }

    /** Enable direct 3D aiming while preserving the stop/recenter state and learned odometry offset. */
    public void setOdometryVisionEnabled(boolean enabled) {
        if (odometryVisionEnabled == enabled) return;
        odometryVisionEnabled = enabled;
        resetAim();
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
            motionStatus = "Manual: waiting for bumpers";
            resetVisionDiagnostics("Manual recenter; auto aim off");
        }
    }

    /** Positive direction turns counterclockwise; both or neither bumper stops motion. */
    public void setManualInput(boolean leftBumper, boolean rightBumper) {
        manualDirection = manualControl && leftBumper != rightBumper
                ? (leftBumper ? 1 : -1) : 0;
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance) {
        initialize(hardwareMap, alliance, 0.0);
    }

    public void initialize(HardwareMap hardwareMap, Alliance alliance, double startingDegrees) {
        this.alliance = Objects.requireNonNull(alliance, "Alliance is required");
        odometryPoseSupplier = null;
        odometryVelocitySupplier = null;
        resetOdometryAimCorrection();
        movingAimDegrees = null;
        movingAimRate = null;
        odometryVisionEnabled = false;
        this.startingDegrees = Double.isFinite(startingDegrees) ? startingDegrees : 0.0;
        encoderZeroTicks = 0;
        turretMotor = hardwareMap.get(DcMotorImplEx.class, "turret");
        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (turret == null) {
            turret = new NextMotor(() -> turretMotor, Units.Radians.of(1.0), POWER_CACHE_TOLERANCE);
        }
        turret.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        limelight.initialize(hardwareMap);
        target = alliance.close;
        lastRecenterDegrees = Double.NaN;
        recenterCount = 0;
        stopImmediately();
    }

    public void stopImmediately() {
        enabled = false;
        manualControl = false;
        manualDirection = 0;
        mode = "OFF";
        motionStatus = "Auto aim off";
        targetDegrees = Double.NaN;
        clearVision();
        resetVisionDiagnostics("Auto aim off");
        pid.reset();
        applyPower(0);
    }

    /** Drop the current setpoint and camera lock so the next loop re-acquires from scratch. */
    private void resetAim() {
        clearVision();
        angleEstimator.resetStillness();
        pid.reset();
        targetDegrees = Double.NaN;
        applyPower(0);
    }

    private void clearVision() {
        visionTimestamp = Double.NaN;
        visionReceiptTimestamp = Long.MIN_VALUE;
        visionDegrees = Double.NaN;
        visionTagId = -1;
        visionSettled = false;
    }

    private void resetVisionDiagnostics(String status) {
        visionStatus = status;
        visionFrameAgeMs = Double.NaN;
        visionPipeline = -1;
        visionTags = "[]";
        visionErrorDegrees = Double.NaN;
        visionRequestedDegrees = Double.NaN;
        visionErrorTagId = -1;
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
            motionStatus = power != 0 ? "Manual bumper control"
                    : manualDirection == 0 ? "Manual: waiting for bumpers"
                    : "Manual power/config invalid or zero";
            String blocked = !Double.isFinite(current) ? "Invalid encoder angle"
                    : limitBlock(current, power);
            if (blocked != null) {
                power = 0;
                motionStatus = blocked;
            }
            applyPower(power);
            return;
        }
        if (!enabled || !calibrated()) {
            mode = !enabled ? "OFF" : "CALIBRATION REQUIRED";
            visionStatus = !enabled ? "Auto aim off" : "Turret calibration required";
            motionStatus = visionStatus;
            pid.reset();
            applyPower(0);
            return;
        }
        double current = getDegrees();
        if (movingAimDegrees != null) {
            clearVision();
            mode = "SHOOTING ON THE MOVE";
            targetDegrees = Calculations.reachableAngle(movingAimDegrees.getAsDouble(), current,
                    TurretConfig.minDegrees, TurretConfig.maxDegrees);
            visionStatus = Double.isFinite(targetDegrees) ? "Ballistic odometry aim"
                    : "No ballistic solution / target outside turret limits";
        } else if (odometryPoseSupplier != null) {
            updateOdometryAim(current);
        } else {
            mode = "LIMELIGHT ONLY: WAITING";
            updateVision(current, null);
        }
        if (!Double.isFinite(targetDegrees)) {
            motionStatus = visionStatus;
            clearVision();
            pid.reset();
            applyPower(0);
            return;
        }
        double velocityDegreesPerSecond = getVelocityDegreesPerSecond();
        double output = calculatePower(targetDegrees - current, velocityDegreesPerSecond);
        motionStatus = visionSettled ? "Vision settled; motor braking"
                : visionTagId >= 0 && isAligned() && !isVisionAligned()
                        ? "Encoder target reached; camera still off"
                : isAligned() ? "Encoder aligned; PID holding" : "Aiming";
        if (!Double.isFinite(output)) motionStatus = "Invalid PID output; stopped";
        else if (output == 0 && !isAligned()) motionStatus = "Zero PID output";
        String blocked = limitBlock(current, output);
        if (blocked != null) {
            output = 0;
            motionStatus = blocked;
        }
        applyPower(output);
    }

    private void updateOdometryAim(double current) {
        boolean wasUsingVision = visionTagId >= 0;
        Pose pose = odometryPoseSupplier.get();
        aimUpdateTimeNanos = System.nanoTime();
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading()) || !Double.isFinite(current)) {
            angleEstimator.resetStillness();
            mode = odometryVisionEnabled ? "VISION + ODOMETRY: INVALID POSE" : "ODOMETRY: INVALID POSE";
            visionStatus = "Invalid drivetrain pose / turret encoder";
            if (odometryVisionEnabled && Double.isFinite(current)) {
                updateVision(current, null);
                if (visionErrorTagId < 0) angleEstimator.tagLost();
            } else {
                clearVision();
            }
            if (wasUsingVision != (visionTagId >= 0)) pid.reset();
            return;
        }
        Velocity velocity = odometryVelocitySupplier.get();
        angleEstimator.updateStillness(aimUpdateTimeNanos,
                velocity == null ? Double.NaN : Math.hypot(velocity.vx, velocity.vy),
                velocity == null ? Double.NaN : Math.toDegrees(velocity.omega),
                getVelocityDegreesPerSecond());
        updateOdometryTarget(current, pose);
        if (odometryVisionEnabled) {
            updateVision(current, pose);
            if (visionErrorTagId < 0) angleEstimator.tagLost();
        }
        if (odometryVisionEnabled && visionTagId < 0) {
            mode = Double.isFinite(targetDegrees)
                    ? (angleEstimator.hasCorrection()
                            ? "ODOMETRY FALLBACK + SAVED CORRECTION" : "ODOMETRY FALLBACK")
                    : "VISION + ODOMETRY: NO REACHABLE TARGET";
        }
        if (wasUsingVision != (visionTagId >= 0)) pid.reset();
    }

    /** The alliance hive whose four-tag cluster includes this ID, or null. */
    private HiveTarget cellForTag(int id) {
        return alliance.close.acceptsTag(id) ? alliance.close
                : alliance.far.acceptsTag(id) ? alliance.far : null;
    }

    private static double odometryBearing(Pose pose, HiveTarget target) {
        return pose == null ? Double.NaN
                : Calculations.bearing(pose.x(), pose.y(), pose.heading(), target);
    }

    private void updateOdometryTarget(double current, Pose pose) {
        mode = "ODOMETRY";
        // Preserve the cached camera setpoint between reads of the same frame.
        if (!odometryVisionEnabled) clearVision();
        if (pose == null) {
            visionStatus = "No drivetrain pose";
            return;
        }
        // Odometry chooses the goal every loop; vision only refines that goal's aim.
        HiveTarget selected = alliance.targetForY(pose.y(), TurretConfig.halfwayY);
        if (selected == null) {
            visionStatus = "Invalid pose / target selection";
            clearVision();
            return;
        }
        if (selected != target) {
            target = selected;
            clearVision();
            pid.reset();
        }
        updateOdometrySetpoint(current, pose);
        visionStatus = Double.isFinite(targetDegrees)
                ? (odometryVisionEnabled ? "Odometry ready; evaluating vision" : "Vision disabled; odometry aim")
                : "Odometry target outside turret limits / invalid pose";
    }

    private void updateOdometrySetpoint(double current, Pose pose) {
        // Convert the chassis-relative goal bearing to the encoder's reference.
        targetDegrees = Calculations.reachableAngle(
                odometryBearing(pose, target) - angleEstimator.getOffsetDegrees(), current,
                TurretConfig.minDegrees, TurretConfig.maxDegrees);
    }

    double calculatePower(double error, double velocityDegreesPerSecond) {
        boolean moving = movingAimDegrees != null;
        boolean trackingVision = visionTagId >= 0;
        gains.kP = trackingVision ? TurretConfig.visionP : TurretConfig.p;
        gains.kI = TurretConfig.i;
        gains.kD = TurretConfig.d;
        if (!Double.isFinite(error)) {
            visionSettled = false;
            pid.reset();
            return Double.NaN;
        }
        if (trackingVision) {
            double settle = TurretConfig.visionSettleDegrees;
            double resume = TurretConfig.visionResumeDegrees;
            if (!Double.isFinite(velocityDegreesPerSecond) || !Double.isFinite(settle)
                    || !Double.isFinite(resume) || settle < 0 || resume < settle) {
                visionSettled = false;
                pid.reset();
                return Double.NaN;
            }
            // Separate enter/exit bands prevent noisy frames from repeatedly reversing kS.
            double threshold = visionSettled ? resume : settle;
            visionSettled = Math.abs(error) <= threshold
                    && Double.isFinite(visionErrorDegrees)
                    && Math.abs(visionErrorDegrees) <= threshold;
            if (visionSettled) {
                pid.reset();
                return 0;
            }
        } else {
            visionSettled = false;
        }
        // Camera updates move the reference abruptly. D should damp encoder motion,
        // rather than amplify those reference jumps as if the turret had moved.
        double rate = moving ? movingAimRate.getAsDouble() : 0;
        if (moving && (!Double.isFinite(rate) || !Double.isFinite(velocityDegreesPerSecond)
                || !Double.isFinite(TurretConfig.movingVelocityFeedforward)
                || TurretConfig.movingVelocityFeedforward < 0)) {
            pid.reset();
            return Double.NaN;
        }
        double output = moving
                ? pid.calculateFromReference(error, 0, rate, velocityDegreesPerSecond)
                : trackingVision
                ? pid.calculateFromReference(error, 0, 0, velocityDegreesPerSecond)
                : pid.calculate(error);
        if (moving) output += rate * TurretConfig.movingVelocityFeedforward;
        output += Math.signum(error) * TurretConfig.staticPower;
        // Counterclockwise is positive.
        output *= Math.signum(TurretConfig.ticksPerDegree);
        return Math.max(-TurretConfig.maxPower, Math.min(TurretConfig.maxPower, output));
    }

    private void updateVision(double current, Pose odometryPose) {
        limelight.setPipeline(TurretConfig.pipeline);
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
        updateTagGeometry(result, current, age, odometryPose);
    }

    private void updateTagGeometry(LLResult result, double current, double age, Pose odometryPose) {
        if (result.getPipelineIndex() != TurretConfig.pipeline
                || !Double.isFinite(age) || age < 0 || age > TurretConfig.maxFrameAgeMs) {
            visionStatus = result.getPipelineIndex() != TurretConfig.pipeline
                    ? "3D pipeline mismatch; expected " + TurretConfig.pipeline
                    : "3D frame too old / invalid frame age";
            clearVision();
            return;
        }
        TagAim aim = selectTagAim(result, current, odometryPose != null);
        if (aim == null) {
            clearVision();
            return;
        }
        if (odometryPose != null) correctOdometryAngle(current, age, odometryPose, aim);
        if (target != aim.cell) clearVision();
        if (aim.id != visionTagId) {
            pid.reset();
            visionSettled = false;
        }
        target = aim.cell;
        applyVisionMeasurement(current, result, aim.id, aim.errorDegrees);
        if (odometryPose != null && visionTagId >= 0) mode = "LIMELIGHT 3D + ODOMETRY FALLBACK";
    }

    private TagAim selectTagAim(LLResult result, double current, boolean allowUnreachable) {
        visionStatus = "No tags for " + (odometryPoseSupplier != null ? target : "this alliance");
        boolean newFrame = isNewVisionFrame(result.getTimestamp(), result.getControlHubTimeStamp());
        TagAim best = null;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            TagAim candidate = evaluateTagAim(tag, current, newFrame, allowUnreachable);
            if (candidate == null) continue;
            if (best == null || candidate.rank > best.rank
                    || (candidate.rank == best.rank && candidate.area > best.area)) {
                best = candidate;
            }
        }
        return best;
    }

    private TagAim evaluateTagAim(LLResultTypes.FiducialResult tag, double current,
                                  boolean newFrame, boolean allowUnreachable) {
        int id = tag.getFiducialId();
        HiveTarget cell = cellForTag(id);
        if (cell == null || (odometryPoseSupplier != null && cell != target)) return null;
        HiveAimGeometry.AimPoint point = HiveAimGeometry.fromTag(cell, id, tag.getTargetPoseCameraSpace());
        if (point == null) {
            visionStatus = "Tag " + id + ": no usable 3D pose";
            return null;
        }
        if (!point.upright) {
            visionStatus = "Tag " + id + ": upside-down cell";
            return null;
        }
        double error = point.errorDegrees();
        if (!Double.isFinite(error) || !(Math.abs(error) <= TurretConfig.acquireDegrees)) {
            visionStatus = "Tag " + id + ": invalid aim error / outside acquisition window";
            return null;
        }
        double aimDegrees = current + TurretConfig.txToCcwSign * error;
        if (!Double.isFinite(aimDegrees)) {
            visionStatus = "Tag " + id + ": invalid encoder angle / configuration";
            return null;
        }
        double candidateAim = odometryPoseSupplier != null && !newFrame && id == visionTagId
                ? visionDegrees : aimDegrees;
        boolean reachable = Double.isFinite(Calculations.reachableAngle(candidateAim, current,
                TurretConfig.minDegrees, TurretConfig.maxDegrees));
        if (!reachable && !allowUnreachable) {
            visionStatus = "3D target outside turret limits";
            return null;
        }
        // Reachable aim first, then tracked tag, current cell, and larger image.
        int rank = (reachable ? 3 : 0) + (id == visionTagId ? 2 : cell == target ? 1 : 0);
        double area = Double.isFinite(tag.getTargetArea()) ? Math.max(0, tag.getTargetArea()) : 0;
        return new TagAim(id, cell, error, rank, area);
    }

    private void correctOdometryAngle(double current, double age, Pose odometryPose, TagAim aim) {
        if (cellForTag(aim.id) != target || aim.cell != target) return;
        angleEstimator.observe(aimUpdateTimeNanos, age,
                odometryBearing(odometryPose, target), current,
                TurretConfig.txToCcwSign * aim.errorDegrees);
        // Keep the fallback ready; a usable camera target remains the primary aim.
        updateOdometrySetpoint(current, odometryPose);
    }

    private static final class TagAim {
        final int id;
        final HiveTarget cell;
        final double errorDegrees;
        final int rank;
        final double area;

        TagAim(int id, HiveTarget cell, double errorDegrees, int rank, double area) {
            this.id = id;
            this.cell = cell;
            this.errorDegrees = errorDegrees;
            this.rank = rank;
            this.area = area;
        }
    }

    private void applyVisionMeasurement(double current, LLResult result, int selectedId,
                                        double selectedError) {
        // Keep this camera measurement even if the encoder-based travel limits reject the target.
        visionErrorDegrees = selectedError;
        visionRequestedDegrees = current + TurretConfig.txToCcwSign * selectedError;
        visionErrorTagId = selectedId;
        double timestamp = result.getTimestamp();
        long receiptTimestamp = result.getControlHubTimeStamp();
        if (isNewVisionFrame(timestamp, receiptTimestamp) || selectedId != visionTagId) {
            visionTimestamp = timestamp;
            visionReceiptTimestamp = receiptTimestamp;
            visionDegrees = Calculations.reachableAngle(
                    current + TurretConfig.txToCcwSign * selectedError, current,
                    TurretConfig.minDegrees, TurretConfig.maxDegrees);
        }
        // Repeated reads of one frame hold its setpoint; they do not add tx again.
        if (Double.isFinite(visionDegrees)) {
            visionTagId = selectedId;
            targetDegrees = visionDegrees;
            mode = "LIMELIGHT 3D POI";
            visionStatus = "Tracking cell opening from tag " + selectedId;
        } else {
            visionStatus = "Vision target outside turret limits / invalid correction";
            clearVision();
        }
    }

    private boolean isNewVisionFrame(double timestamp, long receiptTimestamp) {
        // The SDK returns zero when the native timestamp is absent.
        return Double.isFinite(timestamp) && timestamp > 0
                ? timestamp != visionTimestamp : receiptTimestamp != visionReceiptTimestamp;
    }

    /** Status if this power would drive past a travel limit, otherwise null. */
    private static String limitBlock(double current, double power) {
        double motion = power * TurretConfig.ticksPerDegree;
        if (current <= TurretConfig.minDegrees && motion < 0) return "Blocked by encoder MIN limit";
        if (current >= TurretConfig.maxDegrees && motion > 0) return "Blocked by encoder MAX limit";
        return null;
    }

    private boolean calibrated() {
        double[] values = {TurretConfig.ticksPerDegree, TurretConfig.minDegrees,
                TurretConfig.maxDegrees, TurretConfig.p, TurretConfig.visionP,
                TurretConfig.i, TurretConfig.d,
                TurretConfig.staticPower, TurretConfig.maxPower, TurretConfig.toleranceDegrees};
        for (double value : values) if (!Double.isFinite(value)) return false;
        return TurretConfig.ticksPerDegree != 0 && TurretConfig.minDegrees < 0
                && TurretConfig.maxDegrees > 0 && TurretConfig.p >= 0 && TurretConfig.visionP >= 0
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
    /** Estimated physical angle relative to the chassis; getDegrees() remains the raw encoder reference. */
    public double getChassisDegrees() { return angleEstimator.chassisDegrees(getDegrees()); }
    public double getVisionChassisDegrees() { return angleEstimator.getVisionChassisDegrees(); }
    public double getOdometryAngleOffsetDegrees() { return angleEstimator.getOffsetDegrees(); }
    public boolean hasOdometryAimCorrection() { return angleEstimator.hasCorrection(); }
    public String getOdometryCorrectionStatus() {
        if (!TurretAngleEstimator.applyVisionCorrections || angleEstimator.hasCorrection()) {
            return angleEstimator.getStatus();
        }
        if (!odometryVisionEnabled) return "Vision disabled; holding last angle correction";
        if (visionErrorTagId < 0) return "Waiting for a usable 3D tag";
        return angleEstimator.getStatus();
    }
    public double getVelocityDegreesPerSecond() {
        return turretMotor == null || TurretConfig.ticksPerDegree == 0 ? Double.NaN
                : turretMotor.getVelocity() / TurretConfig.ticksPerDegree;
    }
    public Alliance getAlliance() { return alliance; }
    public HiveTarget getTarget() { return target; }
    public double getPower() { return turret == null ? 0 : turret.getThrottle(); }
    public int getVisionTagId() { return visionTagId; }
    public double getVisionErrorDegrees() { return visionErrorDegrees; }
    public double getVisionRequestedDegrees() { return visionRequestedDegrees; }
    public int getVisionErrorTagId() { return visionErrorTagId; }
    public boolean isVisionAligned() {
        return enabled && !manualControl
                && visionErrorTagId >= 0 && Double.isFinite(visionErrorDegrees)
                && Math.abs(visionErrorDegrees) <= TurretConfig.toleranceDegrees;
    }
    public String getMotionStatus() { return motionStatus; }
    /** Encoder reading at the driver's confirmed physical center, immediately before zeroing. */
    public double getLastRecenterDegrees() { return lastRecenterDegrees; }
    public int getRecenterCount() { return recenterCount; }
    public int getEncoderTicks() { return turretMotor == null ? 0 : turretMotor.getCurrentPosition(); }
    public String getMode() { return mode; }
    public String getVisionStatus() { return visionStatus; }
    public double getVisionFrameAgeMs() { return visionFrameAgeMs; }
    public int getVisionPipeline() { return visionPipeline; }
    public double getVisionTimestamp() { return visionTimestamp; }
    public String getVisionTags() { return visionTags; }
    public boolean isAligned() {
        return enabled && Double.isFinite(targetDegrees)
                && Math.abs(targetDegrees - getDegrees()) <= TurretConfig.toleranceDegrees;
    }
}
