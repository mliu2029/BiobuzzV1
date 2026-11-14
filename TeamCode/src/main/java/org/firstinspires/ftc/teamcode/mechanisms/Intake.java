package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ivy.Command;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

public class Intake implements Mechanism {
    private final NextMotor intake = new NextMotor("intake");

    public final Command runIntake = instant(this::restart);

    public void initialize() {
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
        stopImmediately();
    }

    public void restart() {
        setPower(1);
    }

    public void setPower(double power) {
        intake.setThrottle(power);
    }

    public double getVelocity() { return -intake.getEncoderVelocity().into(Units.RadiansPerSecond); }
    public double getPower() { return intake.getThrottle(); }
    public String getStatus() {
        return getPower() > 0 ? "Intaking" : getPower() < 0 ? "Reverse" : "Off";
    }

    public void stopImmediately() {
        setPower(0);
        intake.update();
    }
}
