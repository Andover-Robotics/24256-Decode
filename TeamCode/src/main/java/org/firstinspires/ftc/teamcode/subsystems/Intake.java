package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    private Motor motor;
    private Servo gate;

    // intake configuration
    public static double inPower = 1.0;
    public static double outPower = -1.0;
    public static double storePower = 0.3;

    // gate configuration
    public static double gateOpen = 0.74;
    public static double gateClosed = 0.60;

    private boolean gateOpenStatus = false;

    public Intake(LinearOpMode opMode) {
        motor = new Motor(opMode.hardwareMap, "intake");
        motor.setRunMode(Motor.RunMode.RawPower);
        gate = opMode.hardwareMap.get(Servo.class, "gate");

        closeGate();
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
        gateOpenStatus = true;
    }

    public void closeGate() {
        gate.setPosition(gateClosed);
        gateOpenStatus = false;
    }

    public Action actionResetGate() {
        return new SequentialAction(
                new InstantAction(() -> gate.getController().pwmDisable()),
                new SleepAction(0.25),
                new InstantAction(() -> gate.getController().pwmEnable())
        );
    }

    public void toggleGate() {
        if (gateOpenStatus) {
            closeGate();
        } else {
            openGate();
        }
    }
}
