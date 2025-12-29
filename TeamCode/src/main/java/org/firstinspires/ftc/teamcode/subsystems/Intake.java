package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private Motor motor;
    private Servo gate;

    // intake configuration
    public static double inPower = 1.0;
    public static double outPower = -1.0;
    public static double storePower = 0.15;

    // gate configuration
    public static double gateOpen = 0.61;
    public static double gateClosed = 0.25;

    public Intake(OpMode opMode) {
        motor = new Motor(opMode.hardwareMap, "intake");
        motor.setRunMode(Motor.RunMode.RawPower);
        gate = opMode.hardwareMap.get(Servo.class, "gate");
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
