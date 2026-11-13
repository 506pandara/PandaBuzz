package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.Robot.showTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.util.PDFLController;
import org.firstinspires.ftc.teamcode.config.util.logging.LogType;
import org.firstinspires.ftc.teamcode.config.util.logging.Logger;

/** Single flywheel motor, velocity-controlled with a PDFL controller. */
@Config
public class Launcher {
    private static final double TICKS_PER_REV = 28.0;

    public static double p = 0.003, d = 0, f = 0.1, l = 0, i = 0.00005;
    /** Target flywheel RPM. Set from the shooter lookup table unless Robot.manualShooting is on. */
    public static double targetRpm = 2500;
    /** RPM error allowed for atTarget(). */
    public static double threshold = 200;

    public enum LauncherState { ON, OFF }

    private final MultipleTelemetry telemetry;
    private final DcMotorEx flywheel;
    private final PDFLController controller = new PDFLController(p, d, f, l, i);

    public LauncherState current = LauncherState.OFF;
    public double currentRpm = 0;
    private double activeTarget = 0;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = hardwareMap.get(DcMotorEx.class, "em3");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        controller.reset();
    }

    public void init() {
        setLauncherState(LauncherState.OFF);
        flywheel.setPower(0);
    }

    public void setLauncherState(LauncherState state) {
        current = state;
    }

    public void periodic() {
        activeTarget = current == LauncherState.ON ? targetRpm : 0;

        currentRpm = flywheel.getVelocity() * 60.0 / TICKS_PER_REV;
        controller.updateConstants(p, d, f, l, i);
        controller.update(currentRpm, activeTarget);
        double power = controller.run();

        flywheel.setPower(activeTarget == 0 ? 0 : Math.max(power, 0));

        if (showTelemetry) {
            telemetry.addData("Launcher target", activeTarget);
            telemetry.addData("Launcher RPM", currentRpm);
            telemetry.addData("Launcher at target", atTarget());
            telemetry.addData("Flywheel current", flywheel.getCurrent(CurrentUnit.AMPS));
        }
    }

    /** Dashboard test: spins to targetRpm and shows tuning telemetry. */
    public void periodicTest() {
        setLauncherState(LauncherState.ON);
        periodic();
        telemetry.addData("Launcher target", activeTarget);
        telemetry.addData("Launcher RPM", currentRpm);
        telemetry.addData("Launcher at target", atTarget());
        telemetry.update();
        log();
    }

    public boolean atTarget() {
        return Math.abs(activeTarget - currentRpm) < threshold;
    }

    /** True once the controller has reached the target. */
    public boolean isReady() {
        return controller.done;
    }

    public void log() {
        Logger.logData(LogType.LAUNCHER_TARGET, String.valueOf(activeTarget));
        Logger.logData(LogType.LAUNCHER_VELOCITY, String.valueOf(currentRpm));
        Logger.logData(LogType.LAUNCHER_SETTLED, String.valueOf(controller.done));
        Logger.logData(LogType.LAUNCHER_POWER, String.valueOf(flywheel.getPower()));
    }
}
