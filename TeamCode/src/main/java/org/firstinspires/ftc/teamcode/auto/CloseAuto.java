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

@Autonomous(name = "Decode Close Auto")
@Config
public class CloseAuto extends LinearOpMode {
    // Positions
    public static Pose2d redAllianceStartPose = new Pose2d(60, -48, Math.toRadians(-52));
    public static Pose2d shoot = new Pose2d(32, -32, Math.toRadians(-49));
    public static Vector2d preFirstIntake = new Vector2d(14, -24);
    public static Vector2d firstIntake = new Vector2d(14, -52);
    public static Vector2d preSecondIntake = new Vector2d(-10, -24);
    public static Vector2d secondIntake = new Vector2d(-10, -52);
    public static Vector2d preThirdIntake = new Vector2d(-34, -24);
    public static Vector2d thirdIntake = new Vector2d(-34, -60);
    public static Vector2d gate = new Vector2d(7, -62);

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
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = true))
                // preload
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = false))
                // spike 1
                .strafeToLinearHeading(preFirstIntake, Math.toRadians(-90))
                .strafeToLinearHeading(firstIntake, Math.toRadians(-90))
                // gate
                .strafeToLinearHeading(gate, Math.toRadians(0))
                .waitSeconds(1)
                // shoot
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = true))
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = false))
                // spike 2
                .strafeToLinearHeading(preSecondIntake, Math.toRadians(-90))
                .strafeToLinearHeading(secondIntake, Math.toRadians(-90))
                // shoot
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = true))
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())
                .stopAndAdd(new InstantAction(() -> MecanumDrive.enablePreciseShooting = false))
                // spike 3
                .strafeToLinearHeading(preThirdIntake, Math.toRadians(-90))
                .strafeToLinearHeading(thirdIntake, Math.toRadians(-90))
                // shoot
                .strafeToSplineHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())
                // gate
                .strafeToLinearHeading(new Vector2d(gate.x, gate.y + 15), Math.toRadians(0))
                .build();

        Actions.runBlocking(
                new ParallelAction(
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}
