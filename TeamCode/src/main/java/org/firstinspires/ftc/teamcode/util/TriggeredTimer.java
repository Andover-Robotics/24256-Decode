package org.firstinspires.ftc.teamcode.util;

public class TriggeredTimer {
    private double beginTs = -1.0;
    private double minTimeToTrigger;

    public TriggeredTimer(double minTimeToTrigger) {
        this.minTimeToTrigger = minTimeToTrigger;
    }

    private static double getTimeSeconds() {
        return System.currentTimeMillis() / 1000.0;
    }

    public boolean periodic(boolean condition) {
        if (condition) {
            if (beginTs < 0) {
                beginTs = getTimeSeconds();
            }

            double t = getTimeSeconds() - beginTs;

            return t > minTimeToTrigger;
        } else {
            beginTs = -1.0;
        }

        return false;
    }
}
