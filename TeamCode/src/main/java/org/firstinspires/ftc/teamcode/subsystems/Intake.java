package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Intake {
    private Motor motor;

    public Intake(OpMode opMode) {
        motor = new Motor(opMode.hardwareMap, "intake");
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    public void setVoltage(double voltage) {
        motor.set(voltage);
    }

    public void in() {
        setVoltage(1.0);
    }

    public void out() {
        setVoltage(-1.0);
    }

    public void stop() {
        setVoltage(0.0);
    }
}
