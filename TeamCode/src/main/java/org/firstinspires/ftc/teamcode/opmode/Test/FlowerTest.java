package org.firstinspires.ftc.teamcode.opmode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Flower;

/** Flower tuning: A = up, B = down, or use Flower.manualTarget from the dashboard. */
@TeleOp
public class FlowerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Flower flower = new Flower(hardwareMap, telemetry);
        flower.init();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) flower.setState(Flower.FlowerState.UP);
            if (gamepad1.b) flower.setState(Flower.FlowerState.DOWN);
            flower.periodic();
        }
    }
}
