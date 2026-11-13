package org.firstinspires.ftc.teamcode.opmode.telePOP;

import static org.firstinspires.ftc.teamcode.config.core.Robot.*;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.subsystems.Hood;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.config.util.Timer;
import org.firstinspires.ftc.teamcode.config.util.logging.CSVInterface;

@Config
@TeleOp(name = "TelePOP")
public class TelePOP extends LinearOpMode {
    private static final double TRIGGER_THRESHOLD = .3;
    /** How long gamepad2 square runs the transfer in reverse (s). */
    public static double slowOuttakeTime = .3;

    private Robot robot;
    private GamepadEx g1, g2;
    private final Timer slowOuttakeTimer = new Timer();

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry, Robot.alliance, autoEndPose);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        Timer loopTimer = new Timer();

        waitForStart();
        robot.startTeleOp();
        loopTimer.reset();

        while (opModeIsActive()) {
            long loopTime = loopTimer.getElapsedTime();
            loopTimer.reset();
            g1.readButtons();
            g2.readButtons();

            robot.update();

            intakeControls();
            shooterControls();
            buttonControls();
            robot.drive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robot.getAlliance() == Alliance.RED ? 0 : Math.PI
            );

            robot.periodic();

            if (showLoopTimes) telemetry.addData("Loop Time (ms)", loopTime);
            if (showTelemetry) addTelemetry();
            if (showTelemetry || showLoopTimes) telemetry.update();
        }
        CSVInterface.log();
    }

    /**
     * The transfer defaults to off each loop. While gamepad1 right bumper (spin-up) is held
     * it keeps its last state.
     */
    private void intakeControls() {
        Intake intake = robot.intake;
        if (!gamepad1.right_bumper)
            intake.setTransferState(Intake.TransferState.OFF);

        // Triggers: intake / outtake
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            intake.setGateState(Intake.GateState.CLOSED);
            intake.setTransferState(Intake.TransferState.INTAKE);
        } else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            intake.setTransferState(Intake.TransferState.OUTTAKE);
        }

        // Gamepad 2 square: short reverse pulse on the transfer
        if (g2.wasJustPressed(GamepadKeys.Button.SQUARE)) slowOuttakeTimer.reset();
        if (gamepad2.square && slowOuttakeTimer.getElapsedTimeSeconds() < slowOuttakeTime)
            intake.setTransferState(Intake.TransferState.SLOW_OUTTAKE);
    }

    /** Bumpers spin up the launcher and open the gate. gamepad1 LB / gamepad2 RB (or autoShoot) feed. */
    private void shooterControls() {
        boolean anyBumper = gamepad1.right_bumper || gamepad1.left_bumper || gamepad2.right_bumper || gamepad2.left_bumper;
        robot.launcher.setLauncherState(anyBumper || keepShooterOn ? Launcher.LauncherState.ON : Launcher.LauncherState.OFF);
        if (anyBumper) robot.intake.setGateState(Intake.GateState.OPEN);

        robot.aimFromFixedPose = gamepad2.left_trigger > TRIGGER_THRESHOLD;

        boolean shooting = gamepad1.left_bumper || gamepad2.right_bumper || (autoShoot && robot.isInLaunchZone());
        if (!shooting) return;

        // Close shots feed immediately; far shots wait for the flywheel to reach speed
        boolean canFeed = robot.getDistanceFromGoal() < closeShotDistance || rapidFireFar || robot.launcher.atTarget();
        robot.intake.setTransferState(canFeed ? Intake.TransferState.SHOOT : Intake.TransferState.OFF);
    }

    /** One-shot button presses. */
    private void buttonControls() {
        // Gamepad 1
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) showTelemetry = !showTelemetry;
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) showLoopTimes = !showLoopTimes;
        if (g1.wasJustPressed(GamepadKeys.Button.SQUARE)) robot.flipAlliance();
        if (g1.wasJustPressed(GamepadKeys.Button.A)) robot.relocalizeAtGoal();
        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) keepShooterOn = !keepShooterOn;
        if (g1.wasJustPressed(GamepadKeys.Button.CIRCLE)) Turret.useTurret = !Turret.useTurret;
        if (g1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) robot.flower.toggle();

        // Gamepad 2
        if (g2.wasJustPressed(GamepadKeys.Button.BACK)) robot.flipAlliance();
        if (g2.wasJustPressed(GamepadKeys.Button.A)) robot.hood.setState(Hood.HoodState.DOWN);
        if (g2.wasJustPressed(GamepadKeys.Button.B)) robot.hood.setState(Hood.HoodState.MID);
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) robot.hood.setState(Hood.HoodState.MIDUP);
        if (g2.wasJustPressed(GamepadKeys.Button.X)) robot.hood.setState(Hood.HoodState.UP);
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) robot.nudgeY(-nudgeAmount);
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) robot.nudgeY(nudgeAmount);
    }

    private void addTelemetry() {
        Pose pose = robot.getFollower().pose();
        Pose turretPose = robot.getTurretPose();
        telemetry.addData("Alliance", robot.getAlliance());
        telemetry.addData("Distance From Goal", robot.getDistanceFromGoal());
        telemetry.addData("Robot", "x %.1f  y %.1f  h %.2f", pose.x(), pose.y(), pose.heading());
        telemetry.addData("Turret", "x %.1f  y %.1f", turretPose.x(), turretPose.y());
        telemetry.addData("Goal", "x %.1f  y %.1f", robot.getGoalX(), goalY);
        telemetry.addData("Zone", robot.zone);
        telemetry.addData("X vel", robot.getFollower().velocity().vx);
    }
}
