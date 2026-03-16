package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Autonomous(name = "Nine Ball Far")
@Config
public class NineBall extends LinearOpMode {
    // Positions
    public static Pose2d start = new Pose2d(-70.28 + 15.0 / 2, -24.0 + 14.75 / 2, Math.toRadians(0));
    public static Pose2d preSpike3 = new Pose2d(-36, -39, Math.toRadians(-90));
    public static Pose2d spike3 = new Pose2d(-36, -54, Math.toRadians(-90));
    public static Pose2d preHp = new Pose2d(-40, -70.28 + 14.75 / 2, Math.toRadians(180));
    public static Pose2d hp = new Pose2d(-70.28 + 15.0 / 2 + 2.5, -70.28 + 14.75 / 2, Math.toRadians(180));
    public static Pose2d shoot = new Pose2d(start.position.x + 3, start.position.y, Math.toRadians(0));

    public Bot bot;

    public Action builtAuto = null;

    public void buildAuto() {
        MecanumDrive drive = bot.drive;

        drive.localizer.setPose(Bot.alliance == Bot.Alliance.RED ? start : Bot.mirror(start));
        TrajectoryActionBuilder builder = drive.actionBuilderColor(start, Bot.alliance == Bot.Alliance.BLUE);

        // preload
        builder = builder
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(new InstantAction(() -> Outtake.MANUAL = true))
                .stopAndAdd(new InstantAction(() -> Outtake.MANUAL_VELOCITY = 4400))
                .stopAndAdd(bot.actionShootThreeFar());

        // spike 2
        builder = builder
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(preSpike3, Math.toRadians(-90))
                .splineToSplineHeading(spike3, Math.toRadians(-90))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(shoot, Math.toRadians(180))
                .stopAndAdd(bot.actionShootThreeFar());

        builder = builder
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(preHp, Math.toRadians(180))
                .splineToSplineHeading(hp, Math.toRadians(180))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThreeFar())
                .stopAndAdd(new InstantAction(() -> Outtake.MANUAL = false))
                .strafeToSplineHeading(new Vector2d(shoot.position.x + 8, shoot.position.y), shoot.heading.log());

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