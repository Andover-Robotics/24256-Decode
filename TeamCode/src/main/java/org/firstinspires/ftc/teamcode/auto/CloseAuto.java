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
    public static double SHOOT_HEADING = -52;
    public static Pose2d redAllianceStartPose = new Pose2d(60, -48, Math.toRadians(SHOOT_HEADING));
    public static Pose2d shoot = new Pose2d(30, -30, Math.toRadians(SHOOT_HEADING));
    public static SimplePosition preFirstIntake = new SimplePosition(14, -28);
    public static SimplePosition firstIntake = new SimplePosition(14, -52);
    public static SimplePosition preSecondIntake = new SimplePosition(-10, -28);
    public static SimplePosition secondIntake = new SimplePosition(-10, -56);
    public static SimplePosition preThirdIntake = new SimplePosition(-32, -28);
    public static SimplePosition thirdIntake = new SimplePosition(-32, -60);
    public static SimplePosition gate = new SimplePosition(3, -62);

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
                // preload
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())
                // spike 1
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preFirstIntake.toVector(), Math.toRadians(-90))
                .strafeToLinearHeading(firstIntake.toVector(), Math.toRadians(-90))
                // gate
                .strafeToLinearHeading(gate.toVector(), Math.toRadians(0))
                .waitSeconds(1)
                // shoot
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())
                // spike 2
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preSecondIntake.toVector(), Math.toRadians(-90))
                .strafeToLinearHeading(secondIntake.toVector(), Math.toRadians(-90))
                // shoot
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())
                // spike 3
                .stopAndAdd(new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preThirdIntake.toVector(), Math.toRadians(-90))
                .strafeToLinearHeading(thirdIntake.toVector(), Math.toRadians(-90))
                // shoot
                .stopAndAdd(new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
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
