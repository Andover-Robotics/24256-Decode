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

    public static double GATE_OPEN = 0.0200;
    public static double GATE_CLOSED = 0.1600;
    
    public static boolean AUTO_REVERSE = true;

    private boolean gateOpen = false;

    public static double THREE_EMF_THRESHOLD = 3;
    public static double THREE_EMF_TIME = 0.150;

    private static double JAM_TIME = 0.5;

    private boolean threePossession = false;
    private boolean jammed = false;
    private boolean shouldReverse = false;

    public static double REVERSAL_POWER = -0.3;
    public static double REVERSAL_TIME = 0.150;
    private TriggeredTimer possessionConfirmationTimer;
    private TriggeredTimer jamConfirmationTimer;
    private TriggeredTimer reversalTimer;

    private double setPower;
    private double emfResistance;

    public static double FLYWHEEL_VELOCITY_DROPPING = 100;


    public Intake(LinearOpMode opMode) {
        motor = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        gate = opMode.hardwareMap.get(Servo.class, "gate");

        possessionConfirmationTimer = new TriggeredTimer(THREE_EMF_TIME);
        jamConfirmationTimer = new TriggeredTimer(JAM_TIME);
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

    public double getPower() {
        return setPower;
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

    public boolean isThreePossession() {
        return threePossession;
    }

    public double getEMFResistance() {
        return emfResistance;
    }

    public boolean isJammed() {
        return jammed;
    }

    public void periodic() {
        double voltage = Bot.getInstance().getBatteryVoltage();
        emfResistance = (setPower == 0) ? 0 : voltage * setPower / motor.getCurrent(CurrentUnit.AMPS);

        motor.setPower(setPower);

        threePossession = possessionConfirmationTimer.periodic(setPower == 1.0 && emfResistance < THREE_EMF_THRESHOLD);
        
        if (!AUTO_REVERSE)
            return;

        Bot bot = Bot.getInstance();

        double error = bot.outtake.getTargetVelocity() - bot.outtake.getRealVelocity();
        jammed = jamConfirmationTimer.periodic(bot.isShooting() && Math.abs(error) < FLYWHEEL_VELOCITY_DROPPING);

        if (jammed) {
            shouldReverse = true;
            jamConfirmationTimer.reset();
        }

        if (shouldReverse) {
            motor.setPower(REVERSAL_POWER);
            if (reversalTimer.periodic(true)) {
                reversalTimer.reset();
                shouldReverse = false;
            }
        } else {
            reversalTimer.reset();
        }
    }
}