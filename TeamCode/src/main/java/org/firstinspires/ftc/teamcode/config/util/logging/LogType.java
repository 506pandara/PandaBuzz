package org.firstinspires.ftc.teamcode.config.util.logging;

public enum LogType {
    TURRET_TARGET("turret target", "degrees"),
    LAUNCHER_VELOCITY("current vel", "rpm"),
    LAUNCHER_TARGET("target vel", "rpm"),
    LAUNCHER_SETTLED("launcher done", "boolean"),
    LAUNCHER_POWER("launcher power", "power"),
    ROBOT_X("x", "in"),
    ROBOT_Y("y", "in"),
    ROBOT_HEADING("heading", "degrees");

    private final String header;
    private final String unit;

    LogType(String header, String unit) {
        this.header = header;
        this.unit = unit;
    }

    public String getHeader() {
        return header;
    }

    public String getUnit() {
        return unit;
    }
}
