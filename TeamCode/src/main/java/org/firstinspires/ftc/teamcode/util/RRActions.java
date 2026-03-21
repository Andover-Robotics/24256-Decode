package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;

public class RRActions {

    private static double getTimeSeconds() {
        return System.nanoTime() / 1e9;
    }

    public interface ConditionalFunction {
        boolean condition();
    }

    public static class WaitUntilAction implements Action {
        private ConditionalFunction f;
        private double minTime;
        private double maxTime;
        private double beginTs = -1.0;

        public WaitUntilAction(ConditionalFunction f, double minTime, double maxTime) {
            this.f = f;
            this.minTime = minTime;
            this.maxTime = maxTime;
        }

        public WaitUntilAction(ConditionalFunction f) {
            this(f, 0.0, Double.POSITIVE_INFINITY);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double deltaTs = 0;
            if (beginTs < 0) {
                beginTs = getTimeSeconds();
            } else {
                deltaTs = getTimeSeconds() - beginTs;
            }

            if (deltaTs < minTime) return true;
            if (deltaTs > maxTime) return false;

            return !f.condition();
        }
    }

    public static class IfElseAction implements Action {
        private ConditionalFunction f;
        private Action trueAction;
        private Action falseAction;

        private Boolean condition = null;

        public IfElseAction(ConditionalFunction f, Action trueAction, Action falseAction) {
            this.f = f;
            this.trueAction = trueAction;
            this.falseAction = falseAction;
        }

        public IfElseAction(ConditionalFunction f, Action trueAction) {
            this(f, trueAction, new NullAction());
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (condition == null) {
                condition = f.condition();
            }

            if (condition) {
                return trueAction.run(packet);
            } else {
                return falseAction.run(packet);
            }
        }
    }
}
