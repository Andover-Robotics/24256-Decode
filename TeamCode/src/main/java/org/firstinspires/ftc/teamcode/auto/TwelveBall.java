package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bot;

@Autonomous(name = "Twelve Ball Close")
@Config
public class TwelveBall extends LinearOpMode {
    // Positions
    public static Pose2d preSpike1 = new Pose2d(12, -39, Math.toRadians(-90));
    public static Pose2d spike1 = new Pose2d(12, -48, Math.toRadians(-90));
    public static Pose2d preSpike2 = new Pose2d(-12, -39, Math.toRadians(-90));
    public static Pose2d spike2 = new Pose2d(-12, -54, Math.toRadians(-90));
    public static Pose2d gate = new Pose2d(2, -60, Math.toRadians(0));
    public static Pose2d preSpike3 = new Pose2d(-36, -39, Math.toRadians(-90));
    public static Pose2d spike3 = new Pose2d(-36, -54, Math.toRadians(-90));
    public static Pose2d shoot = new Pose2d(30, -30, Math.toRadians(-53));
    public static Pose2d end = new Pose2d(0, -45, Math.toRadians(0));

    public Bot bot;

    public Action builtAuto = null;

    public void buildAuto() {
        MecanumDrive drive = bot.drive;

        drive.localizer.setPose(Bot.alliance == Bot.Alliance.RED ? Bot.autoStartRedClose : Bot.autoStartBlueClose);
        TrajectoryActionBuilder builder = drive.actionBuilderColor(Bot.autoStartRedClose, Bot.alliance == Bot.Alliance.BLUE);

        // preload
        builder = builder
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree());


        // spike 2
        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(preSpike1, Math.toRadians(-90))
                .splineToSplineHeading(spike1, Math.toRadians(-90))
                .strafeToSplineHeading(gate.position, gate.heading.log())
                .stopAndAdd(new SleepAction(0.5))
                .setTangent(Math.toRadians(90))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .splineToSplineHeading(shoot, Math.toRadians(0))
                .stopAndAdd(bot.actionShootThree());

        // spike 2
        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(preSpike2, Math.toRadians(-90))
                .splineToSplineHeading(spike2, Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .splineToSplineHeading(shoot, Math.toRadians(0))
                .stopAndAdd(bot.actionShootThree());

        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(preSpike3, Math.toRadians(-90))
                .splineToSplineHeading(spike3, Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .splineToSplineHeading(shoot, Math.toRadians(0))
                .stopAndAdd(bot.actionShootThree())
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(end, Math.toRadians(-90));

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