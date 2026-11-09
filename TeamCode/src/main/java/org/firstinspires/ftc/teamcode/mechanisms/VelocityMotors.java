package org.firstinspires.ftc.teamcode.mechanisms;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.units.Units;

final class VelocityMotors {
    private VelocityMotors() { }

    static void setPIDF(NextMotor motor, double p, double i, double d, double f) {
        motor.getVelocityConstants().withP(p).withI(i).withD(d).withV(f);
    }

    static void setVelocity(NextMotor motor, double velocity) {
        if (velocity == 0) {
            stop(motor);
        } else {
            motor.setVelocitySetpoint(Units.RadiansPerSecond.of(velocity));
        }
    }

    static void stop(NextMotor motor) {
        motor.getVelocityPID().reset();
        motor.setThrottle(0);
        motor.update();
    }

    static double velocity(NextMotor motor) {
        return motor.getEncoderVelocity().into(Units.RadiansPerSecond);
    }
}
