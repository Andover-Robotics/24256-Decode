package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

@Config
public class Intake {
    private DcMotor intakeMotor;
    public final double INTAKE_FORWARD_POWER = 1.0;
    public final double INTAKE_REVERSE_POWER = -1.0;

    public enum Direction {
        FORWARD,
        REVERSE,
        STOP
    }


    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
    }

    public void runIntake(Direction direction) {
        if (direction == Direction.FORWARD) {
            intakeMotor.setPower(INTAKE_FORWARD_POWER);
        } else if (direction == Direction.REVERSE) {
            intakeMotor.setPower(INTAKE_REVERSE_POWER);
        } else if (direction == Direction.STOP) {
            intakeMotor.setPower(0);
        }
    }
}
