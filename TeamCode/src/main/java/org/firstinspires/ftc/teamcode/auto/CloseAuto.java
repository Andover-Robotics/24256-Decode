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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.RRActions;

@Autonomous(name = "Close Auto")
@Config

public class CloseAuto extends LinearOpMode {
    public static int numCycles = 4;
    public static Pose2d preSpike1 = new Pose2d(12, -28, Math.toRadians(-90));
    public static Pose2d spike1 = new Pose2d(12, -48, Math.toRadians(-90));
    public static Pose2d preSpike2 = new Pose2d(-12, -35, Math.toRadians(-90));
    public static Pose2d spike2 = new Pose2d(-12, -54, Math.toRadians(-90));
    public static Pose2d gateIntake = new Pose2d(-15, -65, Math.toRadians(-40));
    public static Pose2d preSpike3 = new Pose2d(-36, -35, Math.toRadians(-90));
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
                .splineToSplineHeading(preSpike2, Math.toRadians(-90))
                .splineToSplineHeading(spike2, Math.toRadians(-90))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(shoot, Math.toRadians(0))
                .stopAndAdd(bot.actionShootThree());

        for (int i = 0; i < numCycles; i++) {
            builder = builder
                    .setTangent(Math.toRadians(180))
                    .splineToSplineHeading(gateIntake, Math.toRadians(-90))
                    .stopAndAdd(new RRActions.WaitUntilAction(() -> bot.intake.getPossessionLevel() == Intake.PossessionState.THREE, 0, 2))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(shoot, Math.toRadians(180))
                    .stopAndAdd(bot.actionShootThree());
        }

        // spike 1
        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(preSpike1, Math.toRadians(-90))
                .splineToSplineHeading(spike1, Math.toRadians(-90))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(shoot, Math.toRadians(0))
                .stopAndAdd(bot.actionShootThree());

        builder = builder
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(preSpike3, Math.toRadians(-90))
                .splineToSplineHeading(spike3, Math.toRadians(-90))
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                .setTangent(Math.toRadians(90))
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

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                Bot.switchAlliance();
                builtAuto = null;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                buildAuto();
            }

            bot.addTelemetry();
            telemetry.addData("Auto Built", (builtAuto != null) ? "true" : "false");
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