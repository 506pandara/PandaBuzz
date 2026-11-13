package org.firstinspires.ftc.teamcode.opmode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

/** Turret tuning: holds the turret at Turret.target degrees (set from the dashboard). */
@TeleOp
public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            turret.periodicTest();
        }
    }
}
