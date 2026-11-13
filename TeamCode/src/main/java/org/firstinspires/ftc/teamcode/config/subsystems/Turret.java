package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;

/** Single-servo turret that keeps the shooter pointed at a field target. */
@Config
public class Turret {
    public static boolean useTurret = true;

    /** Servo positions at 0, +90 and -90 degrees. Positions in between are linearly interpolated. */
    public static double zeroPos = .505, ninetyPos = .695, negativeNinetyPos = .324;
    public static double MIN_ANGLE = -90, MAX_ANGLE = 90;
    /** Degrees added to the aim angle. */
    public static double fudgeFactor = 0;
    public static boolean reversed = false;

    /** Turret angle target in degrees (robot-relative). Set by aim(), or from the dashboard in periodicTest(). */
    public static double target = 0;
    /** Raw servo position for periodicRawTest(). */
    public static double testPos = .5;

    private final MultipleTelemetry telemetry;
    private final Servo servo;

    private double targetX, targetY;
    private Pose turretPose = Pose.zero();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.get(Servo.class, "cs3");
        if (reversed) servo.setDirection(Servo.Direction.REVERSE);
    }

    /** Sets the field point to aim at and the turret's current field pose. */
    public void updateAiming(double targetX, double targetY, Pose turretPose) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.turretPose = turretPose;
    }

    public void periodic() {
        aim();
        servo.setPosition(angleToPosition(target));
    }

    /** Dashboard test: holds the turret at the dashboard 'target' angle. */
    public void periodicTest() {
        servo.setPosition(angleToPosition(target));
        telemetry.addData("Turret target (deg)", target);
        telemetry.addData("Turret servo", servo.getPosition());
        telemetry.update();
    }

    /** Dashboard test: drives the servo to the raw position testPos. */
    public void periodicRawTest() {
        servo.setPosition(testPos);
        telemetry.addData("Turret servo", servo.getPosition());
        telemetry.update();
    }

    private void aim() {
        if (!useTurret) {
            target = 0;
            return;
        }
        double dx = targetX - turretPose.x();
        double dy = targetY - turretPose.y();
        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double relativeAngle = wrapTo180(angleToTarget - Math.toDegrees(turretPose.heading()) + fudgeFactor);

        target = Range.clip(relativeAngle, MIN_ANGLE, MAX_ANGLE);

        if (showTelemetry)
            telemetry.addData("Turret target (deg)", target);
    }

    private double angleToPosition(double degrees) {
        if (degrees >= 0)
            return zeroPos + (degrees / 90.0) * (ninetyPos - zeroPos);
        return zeroPos + (degrees / -90.0) * (negativeNinetyPos - zeroPos);
    }

    private static double wrapTo180(double angle) {
        angle %= 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    public void log() {
        Logger.logData(LogType.TURRET_TARGET, String.valueOf(target));
    }
}
