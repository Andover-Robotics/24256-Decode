package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    private Motor motor;
    public final double INTAKE_FORWARD_POWER = 1.0;
    public final double INTAKE_REVERSE_POWER = -1.0;

    public enum Direction {
        FORWARD,
        REVERSE,
        STOP
    }

    public Intake(OpMode opMode) {
        motor = new Motor(opMode.hardwareMap, "intake");
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void runIntake(Direction direction) {
        if (direction == Direction.FORWARD) {
            motor.set(INTAKE_FORWARD_POWER);
        } else if (direction == Direction.REVERSE) {
            motor.set(INTAKE_REVERSE_POWER);
        } else if (direction == Direction.STOP) {
            motor.set(0);
        }
    }
}
