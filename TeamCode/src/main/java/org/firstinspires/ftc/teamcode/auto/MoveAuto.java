package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bot;

@Autonomous(name = "Move Auto")
@Config
public class MoveAuto extends LinearOpMode {
    // Positions
    public static Pose2d start = new Pose2d(-70.28 + 14.75 / 2, -24.0 - 15.0 / 2, Math.toRadians(-90));
    public Bot bot;

    public Action builtAuto = null;

    public void buildAuto() {
        MecanumDrive drive = bot.drive;

        drive.localizer.setPose(Bot.alliance == Bot.Alliance.RED ? start : Bot.mirror(start));
        TrajectoryActionBuilder builder = drive.actionBuilderColor(start, Bot.alliance == Bot.Alliance.BLUE);

        // preload
        builder = builder
                .strafeToSplineHeading(new Vector2d(start.position.x, start.position.y - 5), Math.toRadians(-90));

        builtAuto = builder.build();
    }

    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        Bot.alliance = Bot.Alliance.RED;

        bot.turret.resetEncoder();

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            gp1.readButtons();
            telemetry.addData("Bot Alliance", (Bot.alliance == Bot.Alliance.RED) ? "Red" : "Blue");
            telemetry.addData("Auto Built?", (builtAuto == null) ? "false" : "true");

            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                Bot.switchAlliance();
                builtAuto = null;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                buildAuto();
            }

            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (builtAuto == null) buildAuto();

        Actions.runBlocking(
                new ParallelAction(
                        bot.actionPeriodic(),
                        builtAuto
                )
        );
    }
}