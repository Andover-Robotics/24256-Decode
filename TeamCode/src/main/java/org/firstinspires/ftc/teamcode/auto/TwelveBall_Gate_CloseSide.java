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

@Autonomous(name = "Decode_12 Ball_Gate_Close Side_Auto")
public class TwelveBall_Gate_CloseSide extends LinearOpMode {
    // Positions
    public static Pose2d redAllianceStartPose = new Pose2d(60, -48, Math.toRadians(-45));
    public static Pose2d shoot = new Pose2d(30, -30, Math.toRadians(-45));
    public static Vector2d preFirstIntake = new Vector2d(12, -34);
    public static Vector2d firstIntake = new Vector2d(12, -56);
    public static Vector2d preSecondIntake = new Vector2d(-10, -34);
    public static Vector2d secondIntake = new Vector2d(-10, -60);
    //Estimate for values; need to tune
    public static Vector2d preThirdIntake = new Vector2d(-32, -34);
    public static Vector2d thirdIntake = new Vector2d(-32, -60);

    //Gate estimated value
    public static Vector2d gatePos = new Vector2d(1, -58);

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
                //shoot preload (1-3)
                .afterTime(0.01, new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())
                //first row
                .afterTime(0.01, new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preFirstIntake, Math.toRadians(-85))
                .strafeToLinearHeading(firstIntake, Math.toRadians(-85))
                .waitSeconds(0.5)
                //open gate
                .strafeToLinearHeading(gatePos, Math.toRadians(0))
                .waitSeconds(3)
                //shoot 3 (4-6)
                .afterTime(0.01, new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())
                //second row
                .afterTime(0.01, new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preSecondIntake, Math.toRadians(-85))
                .strafeToLinearHeading(secondIntake, Math.toRadians(-85))
                .waitSeconds(0.5)
                //shoot 3 (7-9)
                .afterTime(0.01, new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())
                //third row
                .afterTime(0.01, new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preThirdIntake, Math.toRadians(-85))
                .strafeToLinearHeading(thirdIntake, Math.toRadians(-85))
                .waitSeconds(0.5)
                //shoot 3 (10-12)
                .afterTime(0.01, new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShoot())

                .build();

        Actions.runBlocking(
                new ParallelAction( /* ends when auto ends */
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}
