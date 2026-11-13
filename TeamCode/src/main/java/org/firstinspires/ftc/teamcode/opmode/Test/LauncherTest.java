package org.firstinspires.ftc.teamcode.opmode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.logging.CSVInterface;

/** Launcher tuning: spins to Launcher.targetRpm (set from the dashboard) and logs to CSV. */
@TeleOp
public class LauncherTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            launcher.periodicTest();
        }
        CSVInterface.log();
    }
}
