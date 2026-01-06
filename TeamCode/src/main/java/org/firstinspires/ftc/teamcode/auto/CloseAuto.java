package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Bot;

@Autonomous(name = "Decode Close Auto")
public class CloseAuto extends LinearOpMode {
    // Positions
    public static Pose2d redAllianceStartPose = new Pose2d(60, -48, Math.toRadians(-45));
    public static Pose2d shoot = new Pose2d(30, -30, Math.toRadians(-45));
    public static Vector2d preFirstIntake = new Vector2d(12, -34);
    public static Vector2d firstIntake = new Vector2d(12, -56);
    public static Vector2d preSecondIntake = new Vector2d(-12, -34);
    public static Vector2d secondIntake = new Vector2d(-12, -60);
    public static Vector2d preThirdIntake = new Vector2d(-24, -34);
    public static Vector2d thirdIntake = new Vector2d(-24, -60);
    public static Vector2d gate = new Vector2d(1, -58);

    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        Bot bot = Bot.getInstance(this);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        Bot.alliance = Bot.Alliance.RED;

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            telemetry.addData("Bot Alliance", (Bot.alliance == Bot.Alliance.RED) ? "Red" : "Blue");

            if (gp1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                Bot.switchAlliance();
            }
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        Pose2d startPose = (Bot.alliance == Bot.Alliance.RED) ? redAllianceStartPose : Bot.mirror(redAllianceStartPose);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action auto = drive.actionBuilderColor(redAllianceStartPose, Bot.alliance == Bot.Alliance.BLUE)
                // preload
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())
                // spike 1
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preFirstIntake, Math.toRadians(-90))
                .strafeToLinearHeading(firstIntake, Math.toRadians(-90))
                .waitSeconds(0.5)
                // gate
                .strafeToLinearHeading(gate, Math.toRadians(0))
                .waitSeconds(3)
                // shoot
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())
                // spike 2
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preSecondIntake, Math.toRadians(-90))
                .strafeToLinearHeading(secondIntake, Math.toRadians(-90))
                .waitSeconds(0.5)
                // shoot
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())
                // spike 3
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preThirdIntake, Math.toRadians(-90))
                .strafeToLinearHeading(thirdIntake, Math.toRadians(-90))
                .waitSeconds(0.5)
                // shoot
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())

                .build();

        Actions.runBlocking(
                new ParallelAction(
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}
