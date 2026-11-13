package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
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

/**
 * Pedro Pathing 3 configuration.
 * <p>
 * All values below are placeholders (not tuned). Run the AutoTune procedures registered in
 * {@link Tuning} and replace each config with the code they generate.
 */
public class Constants {
    // TODO: Mecanum Tuner. Motor names must match the robot configuration.
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("cm2");
        c.backLeftName.set("cm1");
        c.frontRightName.set("cm3");
        c.backRightName.set("cm0");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // TODO: Pinpoint Tuner
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("ci1");
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // TODO: Foresight Tuner
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.1);
        Controller secondaryTranslationalForward = Controller.proportional(0.05);
        Controller primaryTranslationalLateral = Controller.proportional(0.1);
        Controller secondaryTranslationalLateral = Controller.proportional(0.05);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.01));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.01));

        c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.005, 0.005));

        c.maxAchievableForwardVelocity.set(60.0);
        c.maxAchievableStrafeVelocity.set(45.0);
        c.naturalForwardDeceleration.set(40.0);
        c.naturalStrafeDeceleration.set(40.0);
    });

    public static Drivetrain createDrivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    public static Localizer createLocalizer(HardwareMap h) {
        return new PinpointLocalizer(h, localizerConfig);
    }

    public static Algorithm createAlgorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(createLocalizer(h), createDrivetrain(h), createAlgorithm());
    }
}
