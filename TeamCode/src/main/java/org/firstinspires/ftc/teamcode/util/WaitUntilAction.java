package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class WaitUntilAction implements Action {
    public interface ConditionalFunction {
        boolean condition();
    }

    private ConditionalFunction f;

    public WaitUntilAction(ConditionalFunction f) {
        this.f = f;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        return !f.condition();
    }
}
