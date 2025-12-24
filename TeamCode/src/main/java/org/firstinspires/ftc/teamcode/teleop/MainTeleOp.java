package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Outtake;

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

    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = Bot.getInstance(this);
        Bot.alliance = Bot.Alliance.RED;

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            gp1.readButtons();
            double throttle = gp1.getLeftY();
            double strafe = gp1.getLeftX();
            double turn = gp1.getRightX();

            bot.driveRobotCentric(throttle, strafe, turn);
            bot.periodic();

            gp2.readButtons();
            if (gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                bot.intake.in();
            } else if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                bot.intake.out();
            } else {
                bot.intake.stop();
            }

            if (gp2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.intake.openGate();
            } else if (gp2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.intake.closeGate();
            }

            // TODO: Temporary flywheel control
            if (gp2.getButton(GamepadKeys.Button.DPAD_UP)) {
                bot.outtake.setPower(1.0);
            } else {
                bot.outtake.setPower(0.0);
            }

            if (!bot.outtake.isEnabled()) {
                if (gp2.getButton(GamepadKeys.Button.A)) {
                    addAction(bot.actionShoot(Bot.SHOOT_ONE_DELAY));
                } else if (gp2.getButton(GamepadKeys.Button.B)) {
                    addAction(bot.actionShoot(Bot.SHOOT_THREE_DELAY));
                }
            }

            if (gp1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                Bot.switchAlliance();
            }

            if (gp1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                if (Bot.alliance == Bot.Alliance.RED) {
                    bot.setPose(Bot.redResetPose);
                } else {
                    bot.setPose(Bot.blueResetPose);
                }
            }

            handleActions(packet);

            telemetry.addData("Flywheel Target Velocity", bot.outtake.getTargetVelocity());
            telemetry.addData("Flywheel Velocity", bot.outtake.getRealVelocity());
            if (!Outtake.MANUAL) {
                Vector2d hit = bot.outtake.hit;
                if (hit != null) {
                    telemetry.addData("Distance", bot.outtake.goalDistance);
                    telemetry.addData("Hit Pose: ", hit.x + " " + hit.y);
                    packet.fieldOverlay()
                            .setFill("red")
                            .fillCircle(hit.x, hit.y, 5);
                }
            }
            telemetry.addData("Controller #1 Left Stick", gp1.getLeftY());
            telemetry.addData("Controller #2 Left Stick", gp2.getLeftY());

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}
