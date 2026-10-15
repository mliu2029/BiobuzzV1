package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.686121182178888);
        c.yPodOffset.set(3.3337233460794284);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.20438892537943715);
                Controller secondaryTranslationalForward = Controller.proportional(0.07551624655503236);
                Controller primaryTranslationalLateral = Controller.proportional(0.30573334330611646);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1129603010062199);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0181061310641462));
                c.brake.set(Controller.proportionalFeedforward(0.015390211404524269));

                c.headingFeedback.set(Controller.proportional(2.052989985543973));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.034342006363681385, 0.010937644335247146));

                c.linearBrakeCoefficients.set(Matrix.diag(0.026494522569530443, 0.07539059298023884));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0032113779137476987, 0.0017280836618469922));

                c.maxAchievableForwardVelocity.set(60.17761947675666);
                c.maxAchievableStrafeVelocity.set(49.50490180893546);
                c.naturalForwardDeceleration.set(32.66266340573102);
                c.naturalStrafeDeceleration.set(56.39260768932444);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
