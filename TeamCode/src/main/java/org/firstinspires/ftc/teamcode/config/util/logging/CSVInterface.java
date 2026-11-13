package org.firstinspires.ftc.teamcode.config.util.logging;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.StringJoiner;
import java.util.concurrent.BlockingQueue;

/** Writes everything buffered in Logger to /FIRST/&lt;timestamp&gt;.csv, one time/value column pair per LogType. */
public class CSVInterface {

    public static void log() {
        String path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/"
                + new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss", Locale.US).format(new Date()) + ".csv";
        StringBuilder fileContent = new StringBuilder();

        StringJoiner header = new StringJoiner(",");
        for (LogType type : LogType.values())
            header.add("time (s)," + type.getHeader() + " (" + type.getUnit() + ")");
        fileContent.append(header).append("\n");

        List<BlockingQueue<Logger.LogEntry>> data = Logger.getData();
        boolean moreData;
        do {
            moreData = false;
            StringJoiner line = new StringJoiner(",");
            for (BlockingQueue<Logger.LogEntry> queue : data) {
                Logger.LogEntry entry = queue.poll();
                if (entry != null) {
                    line.add(entry.getTimestamp() + "," + entry.getData());
                    moreData = true;
                } else {
                    line.add(" , ");
                }
            }
            fileContent.append(line).append("\n");
        } while (moreData);

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(path))) {
            writer.write(fileContent.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
