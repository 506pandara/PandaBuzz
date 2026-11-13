package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Hood {
    public static double hoodDown = .73;
    public static double hoodMid = .4;
    public static double hoodMidUp = .5;
    public static double hoodUp = .89;

    /** Dashboard override: hold the hood at manualHoodTarget. */
    public static boolean manualTarget = false;
    public static double manualHoodTarget = .73;

    public enum HoodState { UP, MIDUP, MID, DOWN, MANUAL }

    private final MultipleTelemetry telemetry;
    private final Servo hood;
    public HoodState current = HoodState.MANUAL;
    private double target = 0;

    public Hood(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hood = hardwareMap.get(Servo.class, "cs2");
    }

    public void init() {
        setState(HoodState.UP);
    }

    public void setState(HoodState state) {
        current = state;
    }

    /** Sets an exact position (clipped to the hood's range) and switches to MANUAL. */
    public void setTarget(double position) {
        setState(HoodState.MANUAL);
        target = Range.clip(position, hoodDown, hoodUp);
    }

    public void periodic() {
        switch (current) {
            case UP:    target = hoodUp; break;
            case MIDUP: target = hoodMidUp; break;
            case MID:   target = hoodMid; break;
            case DOWN:  target = hoodDown; break;
            case MANUAL: break;
        }
        hood.setPosition(manualTarget ? manualHoodTarget : target);

        if (showTelemetry) {
            telemetry.addData("Hood", hood.getPosition());
            telemetry.addData("Hood state", current);
        }
    }
}
