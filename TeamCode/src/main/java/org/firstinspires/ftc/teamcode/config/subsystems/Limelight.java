package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/** Limelight 3A. Pipeline 0 = AprilTag localization, pipeline 1 = ball detection. */
public class Limelight {
    private static final double METERS_TO_INCHES = 1 / .0254;

    private final Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "ll");
        setPipeline(0);
        limelight.start();
    }

    public void setPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    public List<LLResultTypes.DetectorResult> getDetectorResult() {
        return limelight.getLatestResult().getDetectorResults();
    }

    /** Robot pose in Pedro coordinates from the AprilTag botpose, or Pose.zero() if there is no valid result. */
    public Pose getRobotPosFromTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return Pose.zero();

        Pose3D botPose = result.getBotpose();
        double angle = botPose.getOrientation().getYaw(AngleUnit.DEGREES) + 180;
        if (angle > 180) angle -= 360;

        return new Pose(
                botPose.getPosition().y * METERS_TO_INCHES + 1,
                -botPose.getPosition().x * METERS_TO_INCHES - 1,
                Math.toRadians(angle) + Math.PI / 2
        );
    }
}
