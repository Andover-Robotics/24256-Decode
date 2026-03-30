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

    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -1.0;
    public static double STORE_POWER = 0.2;

    public static double GATE_OPEN = 0.0400;
    public static double GATE_CLOSED = 0.2000;

    private boolean gateOpen = false;

    public enum PossessionState {
        NONE, ONE, TWO, THREE, OVER
    }

    public static double[] EMF_THRESHOLDS = {
        4000, 2000, 1000, 500, 250
    };

    private PossessionState proposedState = PossessionState.NONE;
    private PossessionState realState = PossessionState.NONE;


    public static double EMF_TIME = 0.150;

    private TriggeredTimer possessionConfirmationTimer;

    public static double REVERSAL_POWER = -0.5;
    public static double REVERSAL_TIME = 0.150;
    private boolean shouldReverse = false;
    private TriggeredTimer reversalTimer;

    private double setPower;
    private double emfResistance;

    public Intake(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        gate = opMode.hardwareMap.get(Servo.class, "gate");

        possessionConfirmationTimer = new TriggeredTimer(EMF_TIME);
        reversalTimer = new TriggeredTimer(REVERSAL_TIME);

        closeGate();
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
        gateOpen = true;
    }

    public void closeGate() {
        gate.setPosition(GATE_CLOSED);
        gateOpen = false;
    }

    public Action actionResetGate() {
        return new SequentialAction(
                new InstantAction(() -> gate.getController().pwmDisable()),
                new SleepAction(0.1),
                new InstantAction(() -> gate.getController().pwmEnable()),
                new SleepAction(0.1),
                new InstantAction(this::closeGate)
        );
    }

    public void toggleGate() {
        if (gateOpen) {
            closeGate();
        } else {
            openGate();
        }
    }

    public double getEMFResistance() {
        return emfResistance;
    }

    public PossessionState getPossessionLevel() {
        return realState;
    }

    private static PossessionState classifyResistance(double resistance) {
        PossessionState[] states = PossessionState.values();
        for (int i = 0; i < EMF_THRESHOLDS.length; i++) {
            if (resistance > EMF_THRESHOLDS[i]) {
                return states[i];
            }
        }
        return PossessionState.OVER;
    }
    
    public void periodic() {
        double voltage = Bot.getInstance().getBatteryVoltage();
        emfResistance = voltage * setPower / motor.getCurrent(CurrentUnit.AMPS);

        PossessionState raw = classifyResistance(emfResistance);

        if (raw != proposedState) {
            proposedState = raw;
            possessionConfirmationTimer.reset();
        }

        if (possessionConfirmationTimer.periodic(true)) {
            realState = proposedState;
        }

        if (realState == PossessionState.OVER && !shouldReverse) {
            shouldReverse = true;
            reversalTimer.reset();
        }

        if (shouldReverse) {
            motor.setPower(REVERSAL_POWER);
            if (reversalTimer.periodic(true)) {
                shouldReverse = false;
                reversalTimer.reset();
            }
        } else {
            reversalTimer.reset();
            motor.setPower(setPower);
        }
    }
}