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
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.79139);
        c.yPodOffset.set(-4.62497);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.18253273710076584);
                Controller secondaryTranslationalForward = Controller.proportional(0.06744096899416994);
                Controller primaryTranslationalLateral = Controller.proportional(0.2075054537177061);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07666772050085183);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.020290450261206107));
                c.brake.set(Controller.proportionalFeedforward(0.01724688272202519));

                c.headingFeedback.set(Controller.proportional(2.0070240327046043));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04573894955255714, 0.005474843385155854));

                c.linearBrakeCoefficients.set(Matrix.diag(0.043500382627712574, 0.11084353202339156));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002060360542770957, -2.4449995242729455E-4));

                c.maxAchievableForwardVelocity.set(52.04926461767317);
                c.maxAchievableStrafeVelocity.set(42.85176140491269);
                c.naturalForwardDeceleration.set(51.748631890497634);
                c.naturalStrafeDeceleration.set(57.74401393389643);
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
