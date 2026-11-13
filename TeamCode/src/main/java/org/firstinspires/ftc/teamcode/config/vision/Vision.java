package org.firstinspires.ftc.teamcode.config.vision;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Tracks balls seen by the Limelight detector across frames and builds a pickup path through them. */
@Config
public class Vision {
    // Camera mount relative to robot center (mm). Y is forward, X is right.
    public static double CAMERA_OFFSET_X = 0;
    public static double CAMERA_OFFSET_Y = 200;
    public static double CAMERA_ANGLE_OFFSET = 0;
    /** Robot center to intake contact point (in). The path stops this far short of each ball. */
    public static double INTAKE_REACH = 8;
    /** Frames a ball can go unseen before it's forgotten. */
    public static int FORGET_AFTER_TICKS = 100;
    public static int MAX_BALLS_PER_PATH = 3;

    private final List<VisionBall> balls = new ArrayList<>();

    /** Matches new detections to remembered balls, adds new ones, and forgets stale ones. Pose in inches/radians. */
    public void processCurrentBalls(List<DetectorResult> detections, Pose robotPose) {
        double robotX = inToMm(robotPose.x());
        double robotY = inToMm(robotPose.y());
        List<VisionBall> seen = new ArrayList<>();

        if (detections != null) {
            for (DetectorResult detection : detections) {
                VisionBall match = null;
                for (VisionBall ball : balls) {
                    if (ball.isSameBall(detection)) {
                        match = ball;
                        break;
                    }
                }
                if (match != null) {
                    match.update(detection, robotX, robotY, robotPose.heading());
                } else {
                    match = new VisionBall(detection, robotX, robotY, robotPose.heading());
                    balls.add(match);
                }
                seen.add(match);
            }
        }

        for (VisionBall ball : balls)
            if (!seen.contains(ball)) ball.markUnseen();
        balls.removeIf(ball -> ball.getTicksSinceLastSeen() > FORGET_AFTER_TICKS);
    }

    public List<VisionBall> getBalls() {
        return balls;
    }

    /** Path through the closest balls (up to MAX_BALLS_PER_PATH), or null if none are known. */
    public Path getBallPath(Pose start) {
        if (balls.isEmpty()) return null;

        List<VisionBall> targets = new ArrayList<>(balls);
        targets.sort(Comparator.comparingDouble(VisionBall::getDistance));
        targets = targets.subList(0, Math.min(MAX_BALLS_PER_PATH, targets.size()));

        Path[] segments = new Path[targets.size()];
        Pose from = start;
        for (int i = 0; i < targets.size(); i++) {
            Pose to = intakeTarget(from, targets.get(i));
            segments[i] = line(from, to).tangent();
            from = to;
        }
        return path(segments);
    }

    /** The point INTAKE_REACH short of the ball, along the line from 'from'. */
    private static Pose intakeTarget(Pose from, VisionBall ball) {
        double bx = ball.getAbsoluteX();
        double by = ball.getAbsoluteY();
        double dist = Math.hypot(bx - from.x(), by - from.y());
        if (dist < 1e-6) return new Pose(bx, by);

        double back = INTAKE_REACH / dist;
        return new Pose(bx - (bx - from.x()) * back, by - (by - from.y()) * back);
    }

    static double inToMm(double in) {
        return in * 25.4;
    }

    static double mmToIn(double mm) {
        return mm / 25.4;
    }
}
