package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bot;
import org.firstinspires.ftc.teamcode.util.WaitUntilAction;

@Autonomous(name = "Decode Close Auto")
@Config
public class CloseAuto extends LinearOpMode {
    // Positions
    public static Pose2d startCloseRed = new Pose2d(0, 0, Math.toRadians(0));
    public static Pose2d startFarRed = new Pose2d(0, 0, Math.toRadians(0));
    public static Pose2d preSpike2 = new Pose2d(0, 0, Math.toRadians(-90));
    public static Pose2d spike2 = new Pose2d(0, 0, Math.toRadians(-90));
    public static Pose2d preSpike1 = new Pose2d(0, 0, Math.toRadians(-90));
    public static Pose2d spike1 = new Pose2d(0, 0, Math.toRadians(-90));
    public static Pose2d shoot = new Pose2d(0, 0, Math.toRadians(-90));
    public static Pose2d finalShoot = new Pose2d(0, 0, Math.toRadians(-90));
    public static Pose2d gate = new Pose2d(0, 0, 0);

    public Bot bot;

    // auto configurations
    public boolean startClose = true;
    public int numCycles = 2;

    public Action builtAuto = null;

    public void buildAuto() {
        MecanumDrive drive = bot.drive;

        if (startClose) {
            if (Bot.alliance == Bot.Alliance.RED) {
                drive.localizer.setPose(startCloseRed);
            } else {
                drive.localizer.setPose(Bot.mirror(startCloseRed));
            }
        } else {
            if (Bot.alliance == Bot.Alliance.RED) {
                drive.localizer.setPose(startFarRed);
            } else {
                drive.localizer.setPose(Bot.mirror(startFarRed));
            }
        }

        TrajectoryActionBuilder builder = drive.actionBuilderColor(startClose ? startCloseRed : startFarRed, Bot.alliance == Bot.Alliance.BLUE);

        // preload
        builder = builder
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .setTangent(Math.toRadians(90))
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree());

        // spike 2
        builder = builder
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(preSpike2, Math.toRadians(-90))
                .splineToSplineHeading(spike2, Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(shoot, Math.toRadians(180))
                .stopAndAdd(bot.actionShootThree());

        for (int i = 0; i < numCycles; i++) {
            builder = builder
                    .setTangent(Math.toRadians(0))
                    .splineToSplineHeading(gate, Math.toRadians(-90))
                    .stopAndAdd(new WaitUntilAction(() -> bot.intake.countBalls() == 3, 0, 3))
                    .setTangent(Math.toRadians(90))
                    .splineToSplineHeading(shoot, Math.toRadians(180))
                    .stopAndAdd(bot.actionShootThree());
        }

        // spike 1
        builder = builder
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(preSpike1, Math.toRadians(-90))
                .splineToSplineHeading(spike1, Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(finalShoot, Math.toRadians(180))
                .stopAndAdd(bot.actionShootThree());

        builtAuto = builder.build();
    }

    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        Bot.alliance = Bot.Alliance.RED;

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            telemetry.addData("Bot Alliance", (Bot.alliance == Bot.Alliance.RED) ? "Red" : "Blue");
            telemetry.addData("Auto Built", (builtAuto == null) ? "no" : "yes");

            if (gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                Bot.switchAlliance();
                builtAuto = null;
            }

            if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                buildAuto();
            }

            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        bot.actionPeriodic(),
                        builtAuto
                )
        );
    }
}
