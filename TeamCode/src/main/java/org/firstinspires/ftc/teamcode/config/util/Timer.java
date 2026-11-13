package org.firstinspires.ftc.teamcode.config.util;

/** Millisecond stopwatch. Reads 0 until reset() is called. */
public class Timer {
    private long startTime = 0;
    private boolean started = false;

    public void reset() {
        started = true;
        startTime = System.currentTimeMillis();
    }

    public long getElapsedTime() {
        return started ? System.currentTimeMillis() - startTime : 0;
    }

    public double getElapsedTimeSeconds() {
        return getElapsedTime() / 1000.0;
    }
}
