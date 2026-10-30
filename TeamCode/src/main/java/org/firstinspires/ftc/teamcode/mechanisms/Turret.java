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
    private String visionStatus = "Auto aim off";
    private double visionFrameAgeMs = Double.NaN;
    private int visionPipeline = -1;
    private String visionTags = "[]";
    private double visionNearestErrorDegrees = Double.NaN;

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
        resetVisionDiagnostics("Auto aim off");
        pid.reset();
        applyPower(0);
    }

    private void clearVision() {
        visionTimestamp = Double.NaN;
        visionDegrees = Double.NaN;
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
            Pose pose = drivetrain.getFollower().pose();
            if (pose == null) {
                stopImmediately();
                visionStatus = "No drivetrain pose";
                return;
            }
            HiveTarget selected = alliance.targetForY(pose.y(), TurretConfig.halfwayY);
            if (selected == null) {
                stopImmediately();
                visionStatus = "Invalid pose / target selection";
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
            mode = "ODOMETRY";
        }
        if (!Double.isFinite(targetDegrees)) {
            mode = "TARGET OUT OF RANGE / INVALID POSE";
            visionStatus = "Odometry target unreachable; vision not evaluated";
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
            visionStatus = "Disabled: useVision=false";
            clearVision();
            return;
        }
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
        double sumError = 0;
        int accepted = 0;
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
                    TurretConfig.maxFrameAgeMs, txError, TurretConfig.acquireDegrees)) continue;
            sumError += txError;
            accepted++;
        }
        if (accepted == 0) {
            if (visionPipeline != TurretConfig.pipeline) {
                visionStatus = "Pipeline mismatch; expected " + TurretConfig.pipeline;
            } else if (!Double.isFinite(age) || age < 0) {
                visionStatus = "Invalid frame age";
            } else if (age > TurretConfig.maxFrameAgeMs) {
                visionStatus = "Frame too old; max " + TurretConfig.maxFrameAgeMs + " ms";
            } else if (matching == 0) {
                visionStatus = "No tags for " + target + "; expected "
                        + target.firstTagId + ".." + (target.firstTagId + 3);
            } else {
                visionStatus = "Invalid tx/offset or outside acquisition window: "
                        + TurretConfig.acquireDegrees + " deg";
            }
            clearVision(); // Missing/wrong/stale tag falls back to odometry.
            return;
        }

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
            visionStatus = "Tracking " + accepted + " tag(s)";
        } else {
            visionStatus = "Vision target outside turret limits / invalid correction";
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
