package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.ArrayList;

@TeleOp(name = "Decode TeleOp")
public class MainTeleOp extends LinearOpMode {
    private ArrayList<Action> runningActions = new ArrayList<>();

    public void addAction(Action action) {
        runningActions.add(action);
    }

    public void handleActions(TelemetryPacket packet) {
        ArrayList<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
    }

    public static double DEADZONE_SIZE = 0.1;

    public static double deadzone(double x) {
        if (Math.abs(x) < DEADZONE_SIZE) return 0;
        return x;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        Bot bot = Bot.getInstance(this);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        boolean intakeVibrated = false;
        boolean shooterDisconnectVibrated = false;
        Outtake.MANUAL = false;
        boolean lastLeftTrigger = false;
        boolean lastRightTrigger = false;

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            double throttle = deadzone(gp1.getLeftY());
            double strafe = deadzone(gp1.getLeftX());
            double turn = deadzone(gp1.getRightX()) * 0.7;

            double scalar = 1.0;
            if (gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                scalar = 0.6;
            }

            bot.driveRobotCentric(throttle, strafe, turn, scalar);
            bot.periodic();

            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                bot.intake.in();
            } else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                bot.intake.out();
            } else {
                bot.intake.store();
            }

            gp2.readButtons();
            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.intake.toggleGate();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                addAction(bot.intake.actionResetGate());
            }

            if (gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2 && !lastLeftTrigger) {
                addAction(bot.actionShootThree());
            }
            if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2 && !lastRightTrigger) {
                addAction(bot.actionShootOne());
            }

            lastLeftTrigger = gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2;
            lastRightTrigger = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2;

            if (gp2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.outtake.enable();
            } else if (gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.outtake.disable();
            }

            if (bot.outtake.isShooterMotorDisconnected()) {
                gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                if (!shooterDisconnectVibrated) {
                    gamepad2.rumble(2000);
                    shooterDisconnectVibrated = true;
                }
            } else {
                gamepad2.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                shooterDisconnectVibrated = false;
            }

            if (bot.intake.getPossessionLevel() == Intake.PossessionState.THREE) {
                gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                if (!intakeVibrated) {
                    gamepad1.rumble(500);
                    intakeVibrated = true;
                }
            } else {
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                intakeVibrated = false;
            }



            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                Bot.switchAlliance();
            }

            if (gp1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                bot.resetLocalizerTeleOp();
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (Bot.alliance == Bot.Alliance.RED) {
                    bot.resetLocalizer(Bot.autoStartRedClose);
                } else {
                    bot.resetLocalizer(Bot.autoStartBlueClose);
                }
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.turret.setAdjustable(bot.turret.getAdjustable() + 1);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.turret.setAdjustable(bot.turret.getAdjustable() - 1);
            }

            handleActions(packet);

            bot.addTelemetry();
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
