package org.firstinspires.ftc.teamcode.config.util;

import com.acmerobotics.dashboard.config.Config;

/**
 * P + D + F (constant feedforward) + L (constant push in the error's direction) + I controller.
 * The integral only accumulates while the output is unsaturated and resets when the target is 0
 * or the error changes sign.
 */
@Config
public class PDFLController {
    /** Error within which the controller counts as having reached the target. */
    public static double reachedThreshold = 30;
    /** Resets the derivative history if run() hasn't been called for this long (ms). */
    private static final long STALE_MS = 200;

    private double kP, kD, kF, kL, kI;
    private double deadzone;

    private double target, current;
    private double prevError, totalError;
    /** True while the error is within reachedThreshold of a non-zero target. */
    public boolean done = false;

    private final Timer timer = new Timer();
    private final RingBuffer<Long> timeBuffer = new RingBuffer<>(3, 0L);
    private final RingBuffer<Double> errorBuffer = new RingBuffer<>(3, 0.0);

    public PDFLController(double kP, double kD, double kF, double kL, double kI) {
        updateConstants(kP, kD, kF, kL, kI);
        timer.reset();
    }

    public void updateConstants(double kP, double kD, double kF, double kL, double kI) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
        this.kI = kI;
    }

    /** Below this error, the L and I terms are skipped. */
    public void setDeadZone(double deadZone) {
        this.deadzone = deadZone;
    }

    public void reset() {
        timeBuffer.fill(0L);
        errorBuffer.fill(0.0);
        timer.reset();
    }

    public void update(double current, double target) {
        this.target = target;
        this.current = current;
    }

    public double run() {
        double error = target - current;
        // Integrate only while the error keeps its sign
        double integrandError = (prevError > 0 && error < 0) || (prevError < 0 && error > 0) || error == 0 ? 0 : error;
        if (target == 0)
            totalError = 0;

        long now = timer.getElapsedTime();
        long prevTime = timeBuffer.add(now);
        prevError = errorBuffer.add(error);

        long deltaTime = now - prevTime;
        if (deltaTime > STALE_MS) {
            reset();
            return run();
        }
        double deltaError = error - prevError;

        done = Math.abs(error) < reachedThreshold && target != 0;

        double response = kP * error
                + (deltaTime == 0 ? 0 : kD * deltaError / deltaTime)
                + kF;

        if (Math.abs(error) >= deadzone) {
            response += Math.signum(error) * kL;
            if (Math.abs(response) < 1) {
                totalError += integrandError;
                response += kI * totalError;
            }
        }
        return response;
    }
}
