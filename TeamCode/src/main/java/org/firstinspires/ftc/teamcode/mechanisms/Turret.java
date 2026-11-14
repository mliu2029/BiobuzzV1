package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
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
import java.util.Objects;
import java.util.function.Supplier;
import org.firstinspires.ftc.teamcode.data.HiveTarget;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.Calculations;
import org.firstinspires.ftc.teamcode.data.TurretConfig;
import org.firstinspires.ftc.teamcode.data.VirtualGoalSolver;

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

    // Shooting on the move (virtual goal)
    private Supplier<Follower> followerSource = () -> null;
    private boolean shootOnMove;
    private VirtualGoalSolver.Result sotm;
    private double sotmRobotHeading = Double.NaN;
    private String sotmStatus = "Off";
    private double sotmSpeedInPerSec = Double.NaN;

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

    /** Where to read odometry from for shoot-on-the-move; resolved lazily each loop. */
    public void setFollowerSource(Supplier<Follower> source) {
        followerSource = Objects.requireNonNull(source);
    }

    public void setShootOnMove(boolean enabled) {
        shootOnMove = enabled;
        if (!enabled || !this.enabled) clearShootOnMove();
    }

    public void toggleShootOnMove() {
        setShootOnMove(!shootOnMove);
    }

    public boolean isShootOnMove() { return shootOnMove; }
    public String getShootOnMoveStatus() { return sotmStatus; }
    /** Lead of the virtual goal over the real goal (degrees), or NaN when not active. */
    public double getShootOnMoveLeadDegrees() {
        return sotm == null ? Double.NaN : Math.toDegrees(sotm.leadRadians);
    }
    /** Distance from the turret to the virtual goal (inches), or NaN when not active. */
    public double getShootOnMoveDistanceInches() {
        return sotm == null ? Double.NaN : sotm.distanceInches;
    }
    public double getRobotSpeedInPerSec() { return sotmSpeedInPerSec; }

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
        shootOnMove = TurretConfig.sotmDefaultOn;
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
        clearShootOnMove();
        pid.reset();
        applyPower(0);
    }

    private void clearShootOnMove() {
        sotm = null;
        sotmRobotHeading = Double.NaN;
        sotmSpeedInPerSec = Double.NaN;
        sotmStatus = shootOnMove ? "Idle (auto aim off)" : "Off";
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
        updateShootOnMove();
        if (Double.isFinite(fixedDegrees)) {
            // Fixed angles stay relative to the chassis, even as the robot drives.
            if (fixedDegrees >= TurretConfig.minDegrees && fixedDegrees <= TurretConfig.maxDegrees) {
                targetDegrees = fixedDegrees;
            }
            mode = "POSITION";
            visionStatus = "Fixed angle command; auto aim inactive";
            if (shootOnMove) sotmStatus = "Inactive (fixed angle command)";
        } else if (sotm != null && TurretConfig.sotmOdometryAim) {
            aimAtVirtualGoalFromOdometry(current);
        } else {
            mode = "LIMELIGHT ONLY: WAITING";
            updateVision(current);
            if (sotm != null && Double.isFinite(targetDegrees)) addLeadToVisionTarget();
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

    // ---------------- Shooting on the move ----------------

    /** Solves the virtual goal for this loop from odometry; leaves sotm null when unavailable. */
    private void updateShootOnMove() {
        sotm = null;
        sotmRobotHeading = Double.NaN;
        sotmSpeedInPerSec = Double.NaN;
        if (!shootOnMove) {
            sotmStatus = "Off";
            return;
        }
        Follower follower = followerSource.get();
        if (follower == null) {
            sotmStatus = "No odometry (follower not ready)";
            return;
        }
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        if (pose == null || velocity == null) {
            sotmStatus = "No odometry data";
            return;
        }
        if (!sotmConfigValid()) {
            sotmStatus = "Config invalid (see TurretConfig sotm*)";
            return;
        }
        VirtualGoalSolver.Result result = VirtualGoalSolver.solve(
                pose.x(), pose.y(), pose.heading(),
                velocity.vx, velocity.vy, velocity.omega,
                target.x, target.y,
                TurretConfig.sotmTurretOffsetXInches, TurretConfig.sotmTurretOffsetYInches,
                TurretConfig.sotmBallTransferTimeSec, TurretConfig::timeOfFlightSeconds,
                TurretConfig.sotmIterations, Math.toRadians(TurretConfig.sotmMaxLeadDegrees));
        if (result == null) {
            sotmStatus = "No solution (invalid odometry or turret on goal)";
            return;
        }
        sotm = result;
        sotmRobotHeading = pose.heading();
        sotmSpeedInPerSec = Math.hypot(velocity.vx, velocity.vy);
        sotmStatus = result.fallback ? "Fallback: no flight time, no lead"
                : result.leadClamped ? "Active (lead capped)" : "Active";
    }

    private boolean sotmConfigValid() {
        double[] values = {TurretConfig.sotmTurretOffsetXInches, TurretConfig.sotmTurretOffsetYInches,
                TurretConfig.sotmBallTransferTimeSec, TurretConfig.sotmMaxLeadDegrees,
                TurretConfig.sotmVelocityLeadSec};
        for (double value : values) if (!Double.isFinite(value)) return false;
        return TurretConfig.sotmBallTransferTimeSec >= 0 && TurretConfig.sotmMaxLeadDegrees >= 0
                && TurretConfig.sotmIterations >= 1
                && Double.isFinite(TurretConfig.timeOfFlightSeconds(0));
    }

    /** Aim-rate feedforward as extra target degrees (0 when disabled). */
    private double velocityLeadDegrees() {
        return Math.toDegrees(sotm.relativeAngularVelocity * TurretConfig.sotmVelocityLeadSec);
    }

    /** Pure-odometry aiming at the virtual goal, as Seattle Solvers does. */
    private void aimAtVirtualGoalFromOdometry(double current) {
        mode = "SHOOT ON MOVE: ODOMETRY";
        double relative = Calculations.normalize(
                Math.toDegrees(sotm.headingRadians - sotmRobotHeading));
        double reachable = Calculations.reachableAngle(relative + velocityLeadDegrees(), current,
                TurretConfig.minDegrees, TurretConfig.maxDegrees);
        if (Double.isFinite(reachable)) {
            targetDegrees = reachable;
            visionStatus = "Odometry aim at " + target + " virtual goal (Limelight not used)";
        } else {
            visionStatus = "Virtual goal outside turret limits";
        }
    }

    /**
     * Adds the odometry lead on top of the Limelight bearing. The Limelight target is rebuilt from
     * a fresh camera frame each time, so the lead is applied once and never accumulates.
     */
    private void addLeadToVisionTarget() {
        double leaded = targetDegrees + Math.toDegrees(sotm.leadRadians) + velocityLeadDegrees();
        targetDegrees = Math.max(TurretConfig.minDegrees, Math.min(TurretConfig.maxDegrees, leaded));
        mode = "LIMELIGHT + SHOOT ON MOVE";
        visionStatus = visionStatus + String.format(java.util.Locale.US,
                " | lead %.1f deg", Math.toDegrees(sotm.leadRadians));
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
    public double getPower() { return turret == null ? 0 : turret.getThrottle(); }
    public int getVisionTagId() { return visionTagId; }
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
