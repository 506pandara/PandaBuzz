package org.firstinspires.ftc.teamcode.config.util;

/** Distance (in) -> hood position + flywheel RPM, linearly interpolated between tuned points. */
public class ShooterLookup {

    // Distances must be in increasing order
    private static final ShotPoint[] table = {
            new ShotPoint(37, .76, 2150),
            new ShotPoint(48.7, .76, 2200),
            new ShotPoint(58.37, .825, 2350),
            new ShotPoint(67, .833, 2600),
            new ShotPoint(77, .84, 2900),
            new ShotPoint(87.46, .85, 3070),
            new ShotPoint(97.7, .85, 3300),
            new ShotPoint(104, .845, 3400),
            new ShotPoint(109, .845, 3600),
            new ShotPoint(122, .8, 3600),
            new ShotPoint(136, .79, 3600),
            new ShotPoint(147, .79, 3600)
    };

    public static class ShotPoint {
        public final double distance;
        public final double hood;
        public final double rpm;

        public ShotPoint(double distance, double hood, double rpm) {
            this.distance = distance;
            this.hood = hood;
            this.rpm = rpm;
        }
    }

    /** Returns the interpolated shot for a distance, clamped to the ends of the table. */
    public static ShotPoint getShot(double distance) {
        if (distance <= table[0].distance) return table[0];

        for (int i = 0; i < table.length - 1; i++) {
            ShotPoint low = table[i];
            ShotPoint high = table[i + 1];
            if (distance <= high.distance) {
                double t = (distance - low.distance) / (high.distance - low.distance);
                return new ShotPoint(
                        distance,
                        low.hood + t * (high.hood - low.hood),
                        low.rpm + t * (high.rpm - low.rpm)
                );
            }
        }
        return table[table.length - 1];
    }
}
