package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
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

@Autonomous(name = "Decode Far Auto")
@Config
public class FarAuto extends LinearOpMode {
    public static Pose2d redAllianceStartPose = new Pose2d(-62, -24, Math.toRadians(0));
    public static Pose2d shoot = new Pose2d(16, -16, Math.toRadians(-47));
    public static Vector2d preSecondIntake = new Vector2d(-12, -24);
    public static Vector2d secondIntake = new Vector2d(-12, -52);
    public static Vector2d preThirdIntake = new Vector2d(-36, -24);
    public static Vector2d thirdIntake = new Vector2d(-36, -60);
    public static Vector2d end = new Vector2d(-48, -24);

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

        MecanumDrive drive = bot.drive;

        Pose2d startPose = (Bot.alliance == Bot.Alliance.RED) ? redAllianceStartPose : Bot.mirror(redAllianceStartPose);
        drive.localizer.setPose(startPose);

        Action auto = drive.actionBuilderColor(redAllianceStartPose, Bot.alliance == Bot.Alliance.BLUE)
                .stopAndAdd(new InstantAction(() -> bot.outtake.enable()))
                // preload
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = true))
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = false))
                // spike 3
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preThirdIntake, Math.toRadians(-90))
                .strafeToLinearHeading(thirdIntake, Math.toRadians(-90))
                // shoot
                .strafeToLinearHeading(new Vector2d(thirdIntake.x, thirdIntake.y + 30), Math.toRadians(-90))
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = true))
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = false))
                // spike 3
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preSecondIntake, Math.toRadians(-90))
                .strafeToLinearHeading(secondIntake, Math.toRadians(-90))
                // shoot
                .strafeToLinearHeading(new Vector2d(secondIntake.x, secondIntake.y + 15), Math.toRadians(-90))
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = true))
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(bot.actionShootOne())
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = false))
                // end
                .strafeToLinearHeading(end, Math.toRadians(0))
                .build();

        Actions.runBlocking(
                new ParallelAction(
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}