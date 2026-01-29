package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Bot;

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

        bot.outtake.disable();

        boolean intakeVibrated = false;

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

            if (!bot.inShootingMode()) {
                if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                    bot.intake.in();
                } else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                    bot.intake.out();
                } else {
                    bot.intake.store();
                }
            }

            gp2.readButtons();
            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.intake.toggleGate();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                addAction(bot.intake.actionResetGate());
            }

            if (gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                addAction(bot.actionShootThree());
            } else if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                addAction(bot.actionShootOne());
            }

            if (gp2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                bot.outtake.enable();
            } else if (gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                bot.outtake.disable();
            }

            if (bot.intake.ballInIntake()) {
                gamepad1.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
                if (!intakeVibrated) {
                    gamepad1.rumble(500);
                    intakeVibrated = true;
                }
            } else {
                gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                intakeVibrated = false;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                Bot.switchAlliance();
            }

            handleActions(packet);

            telemetry.addData("Bot Alliance", (Bot.alliance == Bot.Alliance.RED) ? "Red" : "Blue");
            telemetry.addData("\nIntake Current", bot.intake.getCurrent());
            telemetry.addData("\nFlywheel Target Velocity", bot.outtake.getTargetVelocity());
            telemetry.addData("Flywheel Velocity", bot.outtake.getRealVelocity());
            if (bot.outtake.hitDistance != null) {
                telemetry.addData("\nHit Distance", bot.outtake.hitDistance);
                telemetry.addData("\nBearing", Math.toDegrees(bot.outtake.bearing));
            }
            telemetry.addData("\nController #1 Left Stick", gp1.getLeftY());
            telemetry.addData("Controller #2 Left Stick", gp2.getLeftY());

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
