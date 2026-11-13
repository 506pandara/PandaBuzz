package org.firstinspires.ftc.teamcode.config.vision;

import static org.firstinspires.ftc.teamcode.config.vision.Vision.mmToIn;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;

/** A ball detected by the Limelight, located on the field from its camera angles. Units are mm internally. */
@Config
public class VisionBall {
    public static double CAMERA_HEIGHT_MM = 185;
    /** Height of the point being tracked (floor contact). */
    public static double TARGET_HEIGHT_MM = 0;
    /** Positive if the camera is tilted downward. */
    public static double CAMERA_PITCH_DEG = 0;
    /** Max angle change (degrees) between frames to count as the same ball. */
    public static double SAME_BALL_ANGLE = 4;

    public enum Color { GREEN, PURPLE, UNKNOWN }

    private final Color color;
    private double tx, ty;
    private double distance;
    private double relativeX, relativeY;
    private double absoluteX, absoluteY;
    private int ticksSinceLastSeen = 0;

    public VisionBall(DetectorResult detection, double robotX, double robotY, double robotHeading) {
        color = classify(detection.getClassName());
        update(detection, robotX, robotY, robotHeading);
    }

    public void update(DetectorResult detection, double robotX, double robotY, double robotHeading) {
        ticksSinceLastSeen = 0;
        tx = detection.getTargetXDegrees();
        ty = detection.getTargetYDegrees();
        updatePosition(robotX, robotY, robotHeading);
    }

    public void markUnseen() {
        ticksSinceLastSeen++;
    }

    private void updatePosition(double robotX, double robotY, double robotHeading) {
        double tanPitch = Math.tan(Math.toRadians(CAMERA_PITCH_DEG + ty));
        if (Math.abs(tanPitch) < 1e-6) return; // target at the horizon, no ground intersection

        // Position relative to the camera on the ground plane
        double camY = (CAMERA_HEIGHT_MM - TARGET_HEIGHT_MM) / tanPitch;
        double camX = camY * Math.tan(Math.toRadians(tx));
        distance = Math.hypot(camX, camY);

        // Rotate by the camera's yaw on the robot, then shift to robot center
        double yaw = Math.toRadians(Vision.CAMERA_ANGLE_OFFSET);
        relativeX = -(camX * Math.cos(yaw) + camY * Math.sin(yaw) + Vision.CAMERA_OFFSET_X);
        relativeY = -(-camX * Math.sin(yaw) + camY * Math.cos(yaw) + Vision.CAMERA_OFFSET_Y);

        // Rotate into the field frame
        double heading = robotHeading - Math.PI / 2;
        absoluteX = robotX + relativeX * Math.cos(heading) - relativeY * Math.sin(heading);
        absoluteY = robotY + relativeX * Math.sin(heading) + relativeY * Math.cos(heading);
    }

    /** Same color and within SAME_BALL_ANGLE of where this ball was last seen. */
    public boolean isSameBall(DetectorResult detection) {
        boolean close = Math.abs(tx - detection.getTargetXDegrees()) < SAME_BALL_ANGLE
                && Math.abs(ty - detection.getTargetYDegrees()) < SAME_BALL_ANGLE;
        return close && color != Color.UNKNOWN && color == classify(detection.getClassName());
    }

    private static Color classify(String name) {
        if ("purple".equalsIgnoreCase(name)) return Color.PURPLE;
        if ("green".equalsIgnoreCase(name)) return Color.GREEN;
        return Color.UNKNOWN;
    }

    public int getTicksSinceLastSeen() { return ticksSinceLastSeen; }
    public double getDistance() { return mmToIn(distance); }
    public double getRelativeX() { return mmToIn(relativeX); }
    public double getRelativeY() { return mmToIn(relativeY); }
    public double getAbsoluteX() { return mmToIn(absoluteX); }
    public double getAbsoluteY() { return mmToIn(absoluteY); }
}
