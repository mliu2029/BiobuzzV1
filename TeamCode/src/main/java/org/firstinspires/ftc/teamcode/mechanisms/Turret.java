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
import java.util.Objects;
import org.firstinspires.ftc.teamcode.data.HiveTarget;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.Calculations;
import org.firstinspires.ftc.teamcode.data.TurretConfig;

public class Turret implements Mechanism {
    //TODO: tune turrent with odo before limelight
    private final Drivetrain drivetrain;
    private final Limelight limelight;

    public Turret(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = Objects.requireNonNull(drivetrain);
        this.limelight = Objects.requireNonNull(limelight);
    }
    private NextMotor turret;
    private DcMotorImplEx turretMotor;
    private final PIDCoefficients gains = new PIDCoefficients(0.05, 0, 0.001);
    private final PIDController pid = new PIDController(gains);
    private HiveTarget target = HiveTarget.RED_CLOSE;
    private Alliance alliance;
    private boolean enabled;
    private double startingDegrees;
    private double targetDegrees = Double.NaN;
    private double fixedDegrees = Double.NaN;
    private double visionTimestamp = Double.NaN;
    private double visionDegrees = Double.NaN;
    private String mode = "OFF";
    private String visionStatus = "NOT RUN";

    public final Command aimTurret = instant(() -> {
        fixedDegrees = Double.NaN;
        targetDegrees = Double.NaN;
        enabled = true;
        clearVision();
        pid.reset();
    });
    public final Command stopTurret = instant(this::stopImmediately);

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
        mode = "OFF";
        targetDegrees = Double.NaN;
        fixedDegrees = Double.NaN;
        clearVision();
        pid.reset();
        applyPower(0);
    }

    private void clearVision() {
        visionTimestamp = Double.NaN;
        visionDegrees = Double.NaN;
    }

    @Override
    public void periodic() {
        if (turret == null) return;
        targetDegrees = Double.NaN;
        visionStatus = "NOT RUN (" + mode + ")";
        if (!enabled || !calibrated()) {
            mode = !enabled ? "OFF" : "CALIBRATION REQUIRED";
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
        } else {
            Pose pose = drivetrain.getFollower().pose();
            if (pose == null) { stopImmediately(); return; }
            HiveTarget selected = alliance.targetForY(pose.y(), TurretConfig.halfwayY);
            if (selected == null) { stopImmediately(); return; }
            if (selected != target) {
                target = selected;
                clearVision();
                pid.reset();
            }
            double bearing = Calculations.bearing(pose.x(), pose.y(), pose.heading(), target);
            targetDegrees = Calculations.reachableAngle(bearing, current,
                    TurretConfig.minDegrees, TurretConfig.maxDegrees);
            mode = "ODOMETRY";
        }
        if (!Double.isFinite(targetDegrees)) {
            mode = "TARGET OUT OF RANGE / INVALID POSE";
            clearVision();
            pid.reset();
            applyPower(0);
            return;
        }
        if (!Double.isFinite(fixedDegrees)) updateVision(current);
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
        if (!TurretConfig.useVision) {
            visionStatus = "OFF (TurretConfig.useVision = false)";
            clearVision();
            return;
        }
        if (!limelight.isAvailable()) {
            visionStatus = "NO LIMELIGHT (hardware config name must be \"limelight\")";
            clearVision();
            return;
        }
        limelight.setPipeline(TurretConfig.pipeline);
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            visionStatus = "NO RESULT (camera not streaming yet)";
            clearVision();
            return;
        }
        if (result.getPipelineIndex() != TurretConfig.pipeline) {
            visionStatus = "WRONG PIPELINE: camera " + result.getPipelineIndex()
                    + ", TurretConfig.pipeline " + TurretConfig.pipeline;
            clearVision();
            return;
        }
        if (!result.isValid() || result.getFiducialResults() == null
                || result.getFiducialResults().isEmpty()) {
            visionStatus = "NO TAGS SEEN (is pipeline " + TurretConfig.pipeline + " AprilTag?)";
            clearVision();
            return;
        }

        double age = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        if (!Double.isFinite(age) || age < 0 || age > TurretConfig.maxFrameAgeMs) {
            visionStatus = String.format("STALE FRAME: %.0f ms > maxFrameAgeMs %.0f",
                    age, TurretConfig.maxFrameAgeMs);
            clearVision();
            return;
        }

        double sumError = 0;
        int accepted = 0;
        StringBuilder seen = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            double tx = tag.getTargetXDegrees();
            seen.append(String.format(" %d(tx %.1f)", id, tx));
            if (!target.acceptsTag(id)) continue;
            double txError = tx - TurretConfig.desiredTx(target, id);
            if (!Calculations.usableVision(target, id,
                    result.getPipelineIndex(), TurretConfig.pipeline, age,
                    TurretConfig.maxFrameAgeMs, txError, TurretConfig.acquireDegrees)) continue;
            sumError += txError;
            accepted++;
        }
        if (accepted == 0) {
            // Missing/wrong/stale tag falls back to odometry.
            visionStatus = "NO USABLE TAG for " + target + " (tags " + target.firstTagId + ".."
                    + (target.firstTagId + 3) + ", |tx - desiredTx| <= " + TurretConfig.acquireDegrees
                    + "); saw" + seen;
            clearVision();
            return;
        }
        visionStatus = String.format("LOCKED: %d tag(s), avg tx error %.1f, age %.0f ms",
                accepted, sumError / accepted, age);

        if (result.getTimestamp() != visionTimestamp) {
            visionTimestamp = result.getTimestamp();
            visionDegrees = Calculations.reachableAngle(
                    current + TurretConfig.txToCcwSign * sumError / accepted, current,
                    TurretConfig.minDegrees, TurretConfig.maxDegrees);
        }
        // Repeated reads of one frame hold its setpoint; they do not add tx again.
        if (Double.isFinite(visionDegrees)) {
            targetDegrees = visionDegrees;
            mode = "LIMELIGHT";
        } else {
            visionStatus = "TAG SEEN BUT CORRECTION OUTSIDE min/maxDegrees";
            clearVision();
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
                : startingDegrees + turretMotor.getCurrentPosition() / TurretConfig.ticksPerDegree;
    }

    public double getTargetDegrees() { return targetDegrees; }
    public int getEncoderTicks() { return turretMotor == null ? 0 : turretMotor.getCurrentPosition(); }
    public String getMode() { return mode; }
    /** Why the last loop did or did not use the Limelight; shows the first check that failed. */
    public String getVisionStatus() { return visionStatus; }
    public HiveTarget getTarget() { return target; }
    public boolean isAligned() {
        return enabled && Double.isFinite(targetDegrees)
                && Math.abs(targetDegrees - getDegrees()) <= TurretConfig.toleranceDegrees;
    }

}
