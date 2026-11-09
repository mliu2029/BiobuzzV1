package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
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
import org.firstinspires.ftc.teamcode.data.HiveTarget;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.Calculations;
import org.firstinspires.ftc.teamcode.data.HiveAimGeometry;
import org.firstinspires.ftc.teamcode.data.TurretConfig;

public class Turret implements Mechanism {
    private static final double POWER_CACHE_TOLERANCE = 0.001;
    private final Limelight limelight;
    private NextMotor turret;
    private DcMotorImplEx turretMotor;
    private final PIDCoefficients gains = new PIDCoefficients(0.01, 0, 0.001);
    private final PIDController pid = new PIDController(gains);
    private HiveTarget target = HiveTarget.RED_CLOSE;
    private Alliance alliance;
    private Supplier<Pose> odometryTestPose;
    private DoubleSupplier movingAimDegrees;
    private DoubleSupplier movingAimRate;
    private boolean testVisionEnabled;
    private int testPointOfInterestTagId = -1;
    private boolean testTagGeometryEnabled;
    private HiveTarget lastGeometryTarget;
    private boolean enabled;
    private boolean manualControl;
    private double manualDirection;
    private double startingDegrees;
    private int encoderZeroTicks;
    private double targetDegrees = Double.NaN;
    private double fixedDegrees = Double.NaN;
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
        }
        manualControl = false;
        manualDirection = 0;
        fixedDegrees = Double.NaN;
        targetDegrees = Double.NaN;
        enabled = true;
        mode = movingAimDegrees != null ? "SHOOTING ON THE MOVE"
                : odometryTestPose == null ? "LIMELIGHT ONLY: WAITING" : "ODOMETRY TEST";
        motionStatus = movingAimDegrees != null ? "Waiting for ballistic solution"
                : odometryTestPose == null ? "Waiting for camera" : "Waiting for pose";
        clearVision();
        resetVisionDiagnostics("Awaiting auto aim update");
        pid.reset();
    }

    public void useOdometryForTest(Supplier<Pose> poseSupplier) {
        if (turretMotor == null) throw new IllegalStateException("Initialize the turret first");
        movingAimDegrees = null;
        movingAimRate = null;
        odometryTestPose = Objects.requireNonNull(poseSupplier);
        resumeAutoAim();
    }

    /** Continuously track a robot-relative ballistic setpoint and its angular velocity. */
    public void useMovingAim(DoubleSupplier angleDegrees, DoubleSupplier rateDegreesPerSecond) {
        if (turretMotor == null) throw new IllegalStateException("Initialize the turret first");
        movingAimDegrees = Objects.requireNonNull(angleDegrees);
        movingAimRate = Objects.requireNonNull(rateDegreesPerSecond);
        odometryTestPose = null;
        resumeAutoAim();
    }

    public boolean isTrackingMovingAim() {
        return enabled && !manualControl && !Double.isFinite(fixedDegrees) && movingAimDegrees != null
                && calibrated() && Double.isFinite(TurretConfig.movingVelocityFeedforward)
                && TurretConfig.movingVelocityFeedforward >= 0;
    }

    /** Allow camera correction with odometry fallback in the test; preserve stop/recenter state. */
    public void setTestVisionEnabled(boolean enabled) {
        if (testVisionEnabled == enabled) return;
        testVisionEnabled = enabled;
        resetAim();
    }

    /** Single-tag pipeline POI test; -1 restores the test's raw-tag vision mode. */
    public void setTestPointOfInterestTag(int tagId) {
        if (tagId < -1) throw new IllegalArgumentException("POI tag must be -1 or a tag ID");
        if (testPointOfInterestTagId == tagId) return;
        testPointOfInterestTagId = tagId;
        lastGeometryTarget = null;
        resetAim();
    }

    /** Project a common cell aim point from any of its four alliance tags in the test. */
    public void setTestTagGeometryEnabled(boolean enabled) {
        if (testTagGeometryEnabled == enabled) return;
        testTagGeometryEnabled = enabled;
        lastGeometryTarget = null;
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
        odometryTestPose = null;
        movingAimDegrees = null;
        movingAimRate = null;
        testVisionEnabled = false;
        testPointOfInterestTagId = -1;
        testTagGeometryEnabled = false;
        lastGeometryTarget = null;
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
        fixedDegrees = Double.NaN;
        clearVision();
        resetVisionDiagnostics("Auto aim off");
        pid.reset();
        applyPower(0);
    }

    /** Drop the current setpoint and camera lock so the next loop re-acquires from scratch. */
    private void resetAim() {
        clearVision();
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
        if (Double.isFinite(fixedDegrees)) {
            // Fixed angles stay relative to the chassis, even as the robot drives.
            if (fixedDegrees >= TurretConfig.minDegrees && fixedDegrees <= TurretConfig.maxDegrees) {
                targetDegrees = fixedDegrees;
            }
            mode = "POSITION";
            visionStatus = "Fixed angle command; auto aim inactive";
        } else if (movingAimDegrees != null) {
            clearVision();
            mode = "SHOOTING ON THE MOVE";
            targetDegrees = Calculations.reachableAngle(movingAimDegrees.getAsDouble(), current,
                    TurretConfig.minDegrees, TurretConfig.maxDegrees);
            visionStatus = Double.isFinite(targetDegrees) ? "Ballistic odometry aim"
                    : "No ballistic solution / target outside turret limits";
        } else if (odometryTestPose != null) {
            updateTestAim(current);
        } else {
            mode = "LIMELIGHT ONLY: WAITING";
            updateVision(current);
        }
        if (!Double.isFinite(targetDegrees)) {
            motionStatus = Double.isFinite(fixedDegrees)
                    ? "Target outside encoder limits" : visionStatus;
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

    private void updateTestAim(double current) {
        boolean wasUsingVision = visionTagId >= 0;
        // Always calculate a fresh odometry setpoint before trying the camera.
        updateOdometryTest(current);
        if (testVisionEnabled) {
            updateVision(current);
            if (visionTagId < 0 && Double.isFinite(targetDegrees)) {
                mode = "ODOMETRY FALLBACK";
            }
        }
        // Avoid carrying derivative/integral history across a change of aiming source.
        if (wasUsingVision != (visionTagId >= 0)) pid.reset();
    }

    /** The alliance hive whose four-tag cluster includes this ID, or null. */
    private HiveTarget cellForTag(int id) {
        return alliance.close.acceptsTag(id) ? alliance.close
                : alliance.far.acceptsTag(id) ? alliance.far : null;
    }

    private void updateOdometryTest(double current) {
        mode = "ODOMETRY TEST";
        // Preserve the cached camera setpoint between reads of the same frame.
        if (!testVisionEnabled) clearVision();
        Pose pose = odometryTestPose.get();
        if (pose == null) {
            visionStatus = "No drivetrain pose";
            return;
        }
        // Keep searching for the cell whose POI is configured, even across the field midpoint.
        boolean poiSelected = testVisionEnabled && testPointOfInterestTagId >= 0;
        HiveTarget selected;
        if (testVisionEnabled && testTagGeometryEnabled) {
            selected = lastGeometryTarget;
            if (selected == null) selected = cellForTag(testPointOfInterestTagId);
            if (selected == null) selected = alliance.targetForY(pose.y(), TurretConfig.halfwayY);
        } else {
            selected = poiSelected ? cellForTag(testPointOfInterestTagId)
                    : alliance.targetForY(pose.y(), TurretConfig.halfwayY);
        }
        if (selected == null) {
            visionStatus = poiSelected ? "POI tag does not belong to this alliance"
                    : "Invalid pose / target selection";
            return;
        }
        if (selected != target) {
            target = selected;
            clearVision();
            pid.reset();
        }
        double bearing = Calculations.bearing(pose.x(), pose.y(), pose.heading(), target);
        targetDegrees = Calculations.reachableAngle(bearing, current,
                TurretConfig.minDegrees, TurretConfig.maxDegrees);
        visionStatus = Double.isFinite(targetDegrees) ? "Vision disabled for odometry test"
                : "Odometry target outside turret limits / invalid pose";
    }

    double calculatePower(double error, double velocityDegreesPerSecond) {
        boolean moving = movingAimDegrees != null && !Double.isFinite(fixedDegrees);
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

    private void updateVision(double current) {
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
        // Camera-only aiming always projects the opening from one tag's full 3D pose.
        if (odometryTestPose == null || (testVisionEnabled && testTagGeometryEnabled)) {
            updateTagGeometry(result, current, age);
            return;
        }
        if (odometryTestPose != null && testVisionEnabled && testPointOfInterestTagId >= 0) {
            updatePointOfInterest(result, current, age);
            return;
        }
        double acquireDegrees = TurretConfig.acquireDegrees;
        // Legacy raw-tag test mode retains its calibrated middle-tag offsets.
        double firstError = Double.NaN;
        double secondError = Double.NaN;
        int matching = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id != target.firstAimTagId && id != target.secondAimTagId) continue;
            matching++;
            double txError = tag.getTargetXDegrees() - TurretConfig.desiredTx(target, id);
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
            clearVision(); // Keep an odometry fallback if the test supplied one.
            return;
        }

        double selectedError = selectedId == target.firstAimTagId ? firstError : secondError;
        applyVisionMeasurement(current, result, selectedId, selectedError,
                odometryTestPose == null ? "LIMELIGHT ONLY" : "ODOMETRY + LIMELIGHT",
                "Tracking tag " + selectedId);
    }

    private void updateTagGeometry(LLResult result, double current, double age) {
        if (result.getPipelineIndex() != TurretConfig.pipeline
                || !Double.isFinite(age) || age < 0 || age > TurretConfig.maxFrameAgeMs) {
            visionStatus = result.getPipelineIndex() != TurretConfig.pipeline
                    ? "3D pipeline mismatch; expected " + TurretConfig.pipeline
                    : "3D frame too old / invalid frame age";
            clearVision();
            return;
        }
        int selectedId = -1;
        HiveTarget selectedCell = null;
        double selectedError = Double.NaN;
        int bestRank = -1;
        double bestArea = -1;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            HiveTarget cell = cellForTag(id);
            if (cell == null) continue;
            HiveAimGeometry.AimPoint point = HiveAimGeometry.fromTag(cell, id,
                    tag.getTargetPoseCameraSpace());
            if (point == null || !point.upright) continue;
            // Individual 3D poses are unaffected by the camera's global POI setting.
            double error = point.errorDegrees(tag.getTargetXDegrees());
            if (!Calculations.usableVision(cell, id, result.getPipelineIndex(),
                    TurretConfig.pipeline, age, TurretConfig.maxFrameAgeMs,
                    error, TurretConfig.acquireDegrees)) continue;
            double angle = Calculations.reachableAngle(current + TurretConfig.txToCcwSign * error,
                    current, TurretConfig.minDegrees, TurretConfig.maxDegrees);
            if (!Double.isFinite(angle)) continue;
            // Retain one usable tag; otherwise prefer the current cell and a larger image.
            int rank = id == visionTagId ? 2 : cell == target ? 1 : 0;
            double area = Double.isFinite(tag.getTargetArea()) ? Math.max(0, tag.getTargetArea()) : 0;
            if (rank > bestRank || (rank == bestRank && area > bestArea)) {
                selectedId = id;
                selectedCell = cell;
                selectedError = error;
                bestRank = rank;
                bestArea = area;
            }
        }
        if (selectedId < 0) {
            visionStatus = "No usable upright 3D tag pose; check Full 3D/tag size/ID filter";
            clearVision();
            return;
        }
        if (target != selectedCell) {
            clearVision();
        }
        if (selectedId != visionTagId) {
            pid.reset();
            visionSettled = false;
        }
        target = selectedCell;
        lastGeometryTarget = selectedCell;
        applyVisionMeasurement(current, result, selectedId, selectedError,
                odometryTestPose == null ? "LIMELIGHT 3D POI" : "ODOMETRY + LIMELIGHT 3D POI",
                "Tracking cell opening from tag " + selectedId);
    }

    private void updatePointOfInterest(LLResult result, double current, double age) {
        HiveTarget selected = cellForTag(testPointOfInterestTagId);
        if (selected == null) {
            visionStatus = "POI tag does not belong to this alliance";
            clearVision();
            return;
        }
        // LLResult has no primary-fiducial ID accessor. Require a single matching detection
        // so pipeline tx cannot silently refer to a different tag with the wrong POI offset.
        if (result.getFiducialResults().size() != 1
                || result.getFiducialResults().get(0).getFiducialId() != testPointOfInterestTagId) {
            visionStatus = "POI requires pipeline ID filter = " + testPointOfInterestTagId;
            clearVision();
            return;
        }
        double error = result.getTx();
        if (!Calculations.usableVision(selected, testPointOfInterestTagId,
                result.getPipelineIndex(), TurretConfig.pipeline, age,
                TurretConfig.maxFrameAgeMs, error, TurretConfig.acquireDegrees)) {
            visionStatus = result.getPipelineIndex() != TurretConfig.pipeline
                    ? "POI pipeline mismatch; expected " + TurretConfig.pipeline
                    : !Double.isFinite(age) || age < 0 ? "Invalid frame age"
                    : age > TurretConfig.maxFrameAgeMs ? "POI frame too old"
                    : "Invalid POI tx or outside acquisition window";
            clearVision();
            return;
        }
        target = selected;
        applyVisionMeasurement(current, result, testPointOfInterestTagId,
                error, "LIMELIGHT POI", "Tracking cell POI from tag " + testPointOfInterestTagId);
    }

    private void applyVisionMeasurement(double current, LLResult result, int selectedId,
                                        double selectedError, String trackingMode, String status) {
        // Keep this camera measurement even if the encoder-based travel limits reject the target.
        visionErrorDegrees = selectedError;
        visionErrorTagId = selectedId;
        double timestamp = result.getTimestamp();
        long receiptTimestamp = result.getControlHubTimeStamp();
        // The SDK returns zero when the legacy native timestamp is absent. In that
        // case use the SDK's receive time so a newer result can refresh the target.
        boolean newFrame = Double.isFinite(timestamp) && timestamp > 0
                ? timestamp != visionTimestamp : receiptTimestamp != visionReceiptTimestamp;
        if (newFrame || selectedId != visionTagId) {
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
            mode = trackingMode;
            visionStatus = status;
        } else {
            visionStatus = "Vision target outside turret limits / invalid correction";
            clearVision();
        }
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
    public double getVelocityDegreesPerSecond() {
        return turretMotor == null || TurretConfig.ticksPerDegree == 0 ? Double.NaN
                : turretMotor.getVelocity() / TurretConfig.ticksPerDegree;
    }
    public HiveTarget getTarget() { return target; }
    public double getPower() { return turret == null ? 0 : turret.getThrottle(); }
    public int getVisionTagId() { return visionTagId; }
    public double getVisionErrorDegrees() { return visionErrorDegrees; }
    public int getVisionErrorTagId() { return visionErrorTagId; }
    public boolean isVisionAligned() {
        return enabled && !manualControl && !Double.isFinite(fixedDegrees)
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
