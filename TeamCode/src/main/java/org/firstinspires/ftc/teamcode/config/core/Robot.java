package org.firstinspires.ftc.teamcode.config.core;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.Zone;
import org.firstinspires.ftc.teamcode.config.pedro.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.Flower;
import org.firstinspires.ftc.teamcode.config.subsystems.Hood;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.config.util.ShooterLookup;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;

/** Owns the follower and subsystems, and handles localization, aiming and shot selection. */
@Config
public class Robot {
    // Dashboard toggles
    public static boolean showTelemetry = false;
    public static boolean showLoopTimes = false;
    public static boolean logData = false;
    public static boolean keepShooterOn = true;
    /** Feed automatically whenever the robot is in a launch zone. */
    public static boolean autoShoot = false;
    /** Feed far shots without waiting for the flywheel to reach speed. */
    public static boolean rapidFireFar = false;
    /** Stop setting RPM and hood from the lookup table (use dashboard RPM and the hood buttons). */
    public static boolean manualShooting = false;
    public static boolean robotCentric = false;

    public static Alliance alliance = Alliance.RED;
    /** Pose handed over from autonomous; TeleOp starts here. */
    public static Pose autoEndPose = new Pose(0, 0, Math.toRadians(90));

    // Field geometry (inches, Pedro coordinates, red side; blue is mirrored where noted)
    public static double goalXRed = 67, goalXBlue = -67, goalY = 67;
    /** Aim points used for shots from beyond farAimDistance. */
    public static double farGoalXRed = 80, farGoalXBlue = -61;
    public static double farAimDistance = 106;
    /** Shots closer than this are fed without waiting for the flywheel. */
    public static double closeShotDistance = 110;
    /** Known pose against the goal used to re-localize. */
    public static Pose goalReset = new Pose(54.52, 38.92, 0);
    /** Turret position used while aiming from a fixed pose. */
    public static Pose constantShot = new Pose(60, 60, Math.toRadians(45));

    /** Turret center relative to robot center along the robot's forward axis (in). */
    public static double turretOffset = -1.5;
    /** Robot footprint used for launch-zone checks (real robot is 9 in; smaller means more of it must be inside). */
    public static double robotLength = 8.5;
    /** How far one pose-nudge button press moves the robot (in). */
    public static double nudgeAmount = .5;

    public final Launcher launcher;
    public final Turret turret;
    public final Hood hood;
    public final Intake intake;
    public final Flower flower;
    private final Follower follower;

    /** When true, the turret aims from constantShot instead of the live robot position. */
    public boolean aimFromFixedPose = false;
    public Zone zone = Zone.NONE;
    private Pose turretPose = Pose.zero();

    public Robot(HardwareMap hw, Telemetry telemetry, Alliance alliance, Pose startPose) {
        for (LynxModule hub : hw.getAll(LynxModule.class))
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        Robot.alliance = alliance;

        follower = Constants.create(hw);
        follower.setPose(startPose);
        follower.update();

        launcher = new Launcher(hw, telemetry);
        turret = new Turret(hw, telemetry);
        hood = new Hood(hw, telemetry);
        intake = new Intake(hw, telemetry);
        flower = new Flower(hw, telemetry);

        hood.init();
        intake.init();
        flower.init();
        launcher.init();
        Logger.first = true;
    }

    /** Puts the follower in manual (TeleOp) drive mode. */
    public void startTeleOp() {
        follower.manual(0, 0, 0);
    }

    /** Updates localization, turret aim and shot settings. Call at the start of every loop. */
    public void update() {
        follower.update();

        Pose pose = follower.pose();
        double heading = pose.heading();
        turretPose = aimFromFixedPose
                ? constantShot.withHeading(heading)
                : new Pose(pose.x() - turretOffset * Math.cos(heading), pose.y() - turretOffset * Math.sin(heading), heading);

        double distance = getDistanceFromGoal();
        boolean red = alliance == Alliance.RED;
        double aimX = distance < farAimDistance ? getGoalX() : (red ? farGoalXRed : farGoalXBlue);
        turret.updateAiming(aimX, goalY, turretPose);

        if (!manualShooting) {
            ShooterLookup.ShotPoint shot = ShooterLookup.getShot(distance);
            Launcher.targetRpm = shot.rpm;
            hood.setTarget(shot.hood);
        }

        if (logData) log();
    }

    /** Writes every subsystem's state to hardware. Call once per loop after setting states. */
    public void periodic() {
        hood.periodic();
        intake.periodic();
        launcher.periodic();
        turret.periodic();
        flower.periodic();
    }

    /** Manual drive. Ignored while a path is being followed so the sticks don't override it. */
    public void drive(double forward, double strafe, double turn, double headingOffset) {
        if (follower.mode() == Follower.Mode.FOLLOW) return;
        if (robotCentric)
            follower.manual(forward, strafe, turn);
        else
            follower.manual(ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading(), headingOffset));
    }

    public Follower getFollower() {
        return follower;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void flipAlliance() {
        alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
    }

    public double getGoalX() {
        return alliance == Alliance.RED ? goalXRed : goalXBlue;
    }

    public Pose getTurretPose() {
        return turretPose;
    }

    public double getDistanceFromGoal() {
        return Math.hypot(getGoalX() - turretPose.x(), goalY - turretPose.y());
    }

    /** Re-localizes to the known pose against the goal (mirrored for blue). */
    public void relocalizeAtGoal() {
        follower.setPose(alliance == Alliance.RED ? goalReset : mirrorToBlue(goalReset));
    }

    /** Shifts the robot's localized y by dy inches. */
    public void nudgeY(double dy) {
        follower.setY(follower.pose().y() + dy);
    }

    /** Mirrors a red-side pose to the blue side (x -> -x, heading -> PI - heading). */
    public static Pose mirrorToBlue(Pose red) {
        return new Pose(-red.x(), red.y(), Math.PI - red.heading());
    }

    /** True if any corner of the robot is inside a launch zone. Also updates {@link #zone}. */
    public boolean isInLaunchZone() {
        Pose pose = follower.pose();
        double cos = Math.cos(pose.heading()), sin = Math.sin(pose.heading());
        double h = robotLength / 2.0;

        boolean close = false, far = false;
        for (int forward = -1; forward <= 1; forward += 2) {
            for (int side = -1; side <= 1; side += 2) {
                double x = pose.x() + h * (forward * cos - side * sin);
                double y = pose.y() + h * (forward * sin + side * cos);
                close |= y > x && y > -x && y > 0;
                far |= y < x - 48 && y < -x - 48 && y < -48;
            }
        }
        zone = close ? Zone.CLOSE : far ? Zone.FAR : Zone.NONE;
        return close || far;
    }

    private void log() {
        Pose pose = follower.pose();
        turret.log();
        launcher.log();
        Logger.logData(LogType.ROBOT_X, String.valueOf(pose.x()));
        Logger.logData(LogType.ROBOT_Y, String.valueOf(pose.y()));
        Logger.logData(LogType.ROBOT_HEADING, String.valueOf(Math.toDegrees(pose.heading())));
    }
}
