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

@Autonomous(name = "Decode No Gate Auto")
public class NoGateAuto extends LinearOpMode {
    // Positions
    public static Pose2d redAllianceStartPose = new Pose2d(-72 + (24 + 384.0 / 2) / 25.4, -24, 0);
    public static Pose2d shoot = new Pose2d(-24, -24, Math.toRadians(225));
    public static Vector2d preFirstIntake = new Vector2d(-12, -28);
    public static Vector2d firstIntake = new Vector2d(-12, -52);
    public static Vector2d preSecondIntake = new Vector2d(12, -28);
    public static Vector2d secondIntake = new Vector2d(12, -56);
    public static Vector2d preThirdIntake = new Vector2d(36, -28);
    public static Vector2d thirdIntake = new Vector2d(36, -56);

    public void runOpMode() throws InterruptedException {
        Bot bot = Bot.getInstance(this);
        MecanumDrive drive = bot.drive;

        GamepadEx gp1 = new GamepadEx(gamepad1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Bot Alliance", (Bot.alliance == Bot.Alliance.RED) ? "Red" : "Blue");

            if (gp1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                Bot.switchAlliance();
            }
        }

        Pose2d startPose = (Bot.alliance == Bot.Alliance.RED) ? redAllianceStartPose : Bot.mirror(redAllianceStartPose);

        Action auto = drive.actionBuilderColor(startPose, Bot.alliance == Bot.Alliance.BLUE)
                // shoot preload
                .afterTime(0.01, new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())

                .build();

        drive.localizer.setPose(startPose);
        Actions.runBlocking(
                new ParallelAction( /* ends when auto ends */
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}
