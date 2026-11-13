package org.firstinspires.ftc.teamcode.opmode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.util.logging.CSVInterface;

/** Spins the launcher to Launcher.targetRpm (dashboard) and feeds whenever it is at speed. */
@TeleOp
public class ShotTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, Robot.alliance, Robot.autoEndPose);
        waitForStart();

        while (opModeIsActive()) {
            robot.launcher.setLauncherState(Launcher.LauncherState.ON);
            robot.intake.setGateState(Intake.GateState.OPEN);
            boolean ready = robot.launcher.isReady();
            robot.intake.setTransferState(ready ? Intake.TransferState.SHOOT : Intake.TransferState.OFF);

            robot.launcher.periodic();
            robot.intake.periodic();
            robot.launcher.log();

            telemetry.addData("Target RPM", Launcher.targetRpm);
            telemetry.addData("RPM", robot.launcher.currentRpm);
            telemetry.addData("Ready", ready);
            telemetry.update();
        }
        CSVInterface.log();
    }
}
