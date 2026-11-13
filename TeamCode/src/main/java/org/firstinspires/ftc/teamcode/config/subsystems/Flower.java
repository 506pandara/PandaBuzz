package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Two servos that move together between an up and a down position. */
@Config
public class Flower {
    // TODO: tune these positions (placeholders)
    public static double leftUp = 0.718, leftDown = .524;
    public static double rightUp = 0, rightDown = 0.406;

    /** Dashboard override: hold each servo at its manual position. */
    public static boolean manualTarget = false;
    public static double manualLeftTarget = .5, manualRightTarget = .5;

    public enum FlowerState { UP, DOWN }

    private final MultipleTelemetry telemetry;
    private final Servo left, right;
    public FlowerState current = FlowerState.DOWN;

    public Flower(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        left = hardwareMap.get(Servo.class, "es4");
        right = hardwareMap.get(Servo.class, "es0");
    }

    public void init() {
        setState(FlowerState.DOWN);
    }

    public void setState(FlowerState state) {
        current = state;
    }

    public void toggle() {
        setState(current == FlowerState.UP ? FlowerState.DOWN : FlowerState.UP);
    }

    public void periodic() {
        boolean up = current == FlowerState.UP;
        left.setPosition(manualTarget ? manualLeftTarget : up ? leftUp : leftDown);
        right.setPosition(manualTarget ? manualRightTarget : up ? rightUp : rightDown);

        if (showTelemetry) {
            telemetry.addData("Flower state", current);
            telemetry.addData("Flower L", left.getPosition());
            telemetry.addData("Flower R", right.getPosition());
        }
    }
}
