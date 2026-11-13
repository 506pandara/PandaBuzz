package org.firstinspires.ftc.teamcode.opmode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Hood;

/** Hood tuning: set Hood.manualTarget / manualHoodTarget from the dashboard. */
@TeleOp
public class HoodTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Hood hood = new Hood(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            hood.periodic();
        }
    }
}
