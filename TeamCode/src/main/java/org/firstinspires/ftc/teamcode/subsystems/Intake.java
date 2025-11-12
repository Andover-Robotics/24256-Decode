package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Intake {
    private Motor motor;
    private SimpleServo gate;

    // intake configuration
    private static double inPower = 1.0;
    private static double outPower = -1.0;
    private static double storePower = 0.15;

    // gate configuration
    private static double gateOpen = 0.0;
    private static double gateClosed = 0.0;

    public Intake(OpMode opMode) {
        motor = new Motor(opMode.hardwareMap, "intake");
        motor.setRunMode(Motor.RunMode.RawPower);
        gate = new SimpleServo(opMode.hardwareMap, "gate", 0, 360, AngleUnit.DEGREES);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void in() {
        setPower(inPower);
    }

    public void out() {
        setPower(outPower);
    }

    public void store() {
        setPower(storePower);
    }

    public void stop() {
        setPower(0.0);
    }

    public void openGate() {
        gate.setPosition(gateOpen);
    }

    public void closeGate() {
        gate.setPosition(gateClosed);
    }
}
