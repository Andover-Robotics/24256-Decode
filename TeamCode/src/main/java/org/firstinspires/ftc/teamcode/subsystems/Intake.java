package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake {
    private DcMotorEx motor;
    private Servo gate;

    private double beginCurrentTs = -1.0;
    private double current;
    private boolean ballInIntake;

    // intake configuration
    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -1.0;
    public static double STORE_POWER = 0.3;

    // gate configuration
    public static double GATE_OPEN = 0.77;
    public static double GATE_CLOSED = 0.67;

    public static double CURRENT_THRESHOLD = 2100; // mA
    public static double MIN_CURRENT_TIME = 0.300 * 1000; // sec

    private boolean gateOpenStatus = false;

    public Intake(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        gate = opMode.hardwareMap.get(Servo.class, "gate");

        closeGate();
    }

    public void setPower(double power) {
        motor.setPower(power);
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

    public double getCurrent() {
        return current;
    }

    public boolean ballInIntake() {
        return ballInIntake;
    }

    public void periodic() {
        // don't update state if we aren't intaking
        if (motor.getPower() != IN_POWER) {
            return;
        }

        current = motor.getCurrent(CurrentUnit.MILLIAMPS);

        if (current > CURRENT_THRESHOLD) {
            if (beginCurrentTs < 0) {
                beginCurrentTs = System.currentTimeMillis();
            }
            double deltaTime = System.currentTimeMillis() - beginCurrentTs;

            if (deltaTime > MIN_CURRENT_TIME) {
                ballInIntake = true;
            }
        } else {
            beginCurrentTs = -1.0;
            ballInIntake = false;
        }
    }
}
