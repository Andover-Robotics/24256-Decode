package org.firstinspires.ftc.teamcode.util;

public class TriggeredTimer {
    private double beginTs = -1.0;
    private double minTimeToTrigger;

    public TriggeredTimer(double minTimeToTrigger) {
        this.minTimeToTrigger = minTimeToTrigger;
    }

    private static double getTimeSeconds() {
        return System.nanoTime() / 1e9;
    }

    public double getElapsedTime() {
        if (beginTs < 0)
            return 0;

        return getTimeSeconds() - beginTs;
    }

    public boolean periodic(boolean condition) {
        if (condition) {
            if (beginTs < 0) {
                beginTs = getTimeSeconds();
            }

            return getElapsedTime()  > minTimeToTrigger;
        } else {
            reset();
        }
        return false;
    }

    public void reset() {
        beginTs = -1.0;
    }
}
