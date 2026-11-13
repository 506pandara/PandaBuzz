package org.firstinspires.ftc.teamcode.config.util.logging;

import org.firstinspires.ftc.teamcode.config.util.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

/** Buffers timestamped values per LogType; CSVInterface writes them to a file. */
public class Logger {

    public static class LogEntry {
        private final String data;
        private final double timestamp;

        public LogEntry(String data, double timestamp) {
            this.data = data;
            this.timestamp = timestamp;
        }

        public String getData() { return data; }
        public double getTimestamp() { return timestamp; }
    }

    private static final Timer logTimer = new Timer();
    private static final List<BlockingQueue<LogEntry>> buffer = new ArrayList<>();
    /** Set to true to restart timestamps from 0 on the next logData call. */
    public static boolean first = true;

    static {
        for (int i = 0; i < LogType.values().length; i++)
            buffer.add(new LinkedBlockingQueue<>());
    }

    public static void logData(LogType logType, String data) {
        if (first) {
            logTimer.reset();
            first = false;
        }
        buffer.get(logType.ordinal()).add(new LogEntry(data, logTimer.getElapsedTimeSeconds()));
    }

    public static List<BlockingQueue<LogEntry>> getData() {
        return buffer;
    }
}
