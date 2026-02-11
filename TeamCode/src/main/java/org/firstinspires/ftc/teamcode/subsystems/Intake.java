package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.TriggeredTimer;

@Config
public class Intake {
    private DcMotorEx motor;
    private Servo gate;

    // intake configuration
    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -1.0;
    public static double STORE_POWER = 0.3;

    // gate configuration
    public static double GATE_OPEN = 0.77;
    public static double GATE_CLOSED = 0.67;

    private DigitalChannel topBB;
    private DigitalChannel middleBB;
    private DigitalChannel bottomBB;

    private boolean gateOpenStatus = false;

    public static double MIN_CURRENT = 3500;
    public static double CURRENT_PULL_TIME = 0.200 * 1000;
    private TriggeredTimer overPossessionTimer;
    private static double REVERSAL_TIME = 0.100 * 1000;
    private boolean shouldReverse = false;
    private TriggeredTimer reversalTimer;

    private boolean overPossession;

    private double setPower;
    private double current;

    public Intake(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        gate = opMode.hardwareMap.get(Servo.class, "gate");

        closeGate();

        overPossessionTimer = new TriggeredTimer(CURRENT_PULL_TIME);
    }

    public void setPower(double power) {
        setPower = power;
    }

    public void in() {
        setPower(IN_POWER);
    }

    public void out() {
        setPower(OUT_POWER);
    }

    public void store() {
        setPower(STORE_POWER);
    }

    public void stop() {
        setPower(0.0);
    }

    public void openGate() {
        gate.setPosition(GATE_OPEN);
        gateOpenStatus = true;
    }

    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
        gateOpenStatus = false;
    }

    public Action actionResetGate() {
        return new SequentialAction(
                new InstantAction(() -> gate.getController().pwmDisable()),
                new SleepAction(0.25),
                new InstantAction(() -> gate.getController().pwmEnable()),
                new SleepAction(0.25),
                new InstantAction(this::closeGate)
        );
    }

    public void toggleGate() {
        if (gateOpenStatus) {
            closeGate();
        } else {
            openGate();
        }
    }

    public boolean getTopBBStatus() {
        return topBB.getState();
    }

    public boolean getMiddleBBStatus() {
        return middleBB.getState();
    }

    public boolean getBottomBBStatus() {
        return bottomBB.getState();
    }

    public int countBalls() {
        int count = 0;
        if (getTopBBStatus())
            count++;
        if (getMiddleBBStatus())
            count++;
        if (getBottomBBStatus())
            count++;
        return count;
    }

    public double getCurrent() {
        return current;
    }

    public void periodic() {
        current = motor.getCurrent(CurrentUnit.MILLIAMPS);
        overPossession = overPossessionTimer.periodic(current > MIN_CURRENT);

        if (shouldReverse) {
            reversalTimer.periodic(true);
            motor.setPower(-1.0);
            if (reversalTimer.getElapsedTime() > REVERSAL_TIME) {
                shouldReverse = false;
            }
        }

        if (overPossession) {
            shouldReverse = true;
            reversalTimer = new TriggeredTimer(0);
        } else {
            motor.setPower(setPower);
        }
    }
}