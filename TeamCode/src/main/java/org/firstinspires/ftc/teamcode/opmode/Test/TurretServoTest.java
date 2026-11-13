package org.firstinspires.ftc.teamcode.opmode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

/** Turret servo tuning: drives the servo to Turret.testPos (set from the dashboard). */
@TeleOp
public class TurretServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            turret.periodicRawTest();
        }
    }
}
