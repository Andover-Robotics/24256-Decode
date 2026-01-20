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

@Autonomous(name = "Decode Far Auto")

public class FarAuto extends LinearOpMode {
    //Estimate values
    public static Pose2d redAllianceStartPose = new Pose2d(-55,-11, Math.toRadians(-24.5));
    //far shooting zone value
    //public static Pose2d shoot = new Pose2d(-50, -15, Math.toRadians(-24.5));
    //close shooting zone value
    public static Pose2d shoot = new Pose2d(30, -30, Math.toRadians(-45));
    //corner balls
    public static Vector2d firstIntake = new Vector2d(-55, -65);
    //backup
    public static Vector2d firstIntakeBackup = new Vector2d(-55, -65);
    //ram balls again into wall
    public static Vector2d firstIntakeRam = new Vector2d(-55, -75);
    public static Vector2d preSecondIntake = new Vector2d(-32, -34);
    public static Vector2d secondIntake = new Vector2d(-32, -60);

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
                .stopAndAdd(bot.actionShootThree())

                //first row (corner balls)
                .afterTime(0.01, new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(firstIntake, Math.toRadians(-95))
                .strafeToLinearHeading(firstIntakeBackup, Math.toRadians(-90))
                .strafeToLinearHeading(firstIntakeRam, Math.toRadians(-90))
                .waitSeconds(0.5)

                //shoot first row (4-6)
                .afterTime(0.01, new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())

                //second row
                .afterTime(0.01, new InstantAction(() -> bot.intake.in()))
                .strafeToLinearHeading(preSecondIntake, Math.toRadians(-85))
                .strafeToLinearHeading(secondIntake, Math.toRadians(-85))
                .waitSeconds(0.5)

                //shoot 3 (7-9)
                .afterTime(0.01, new InstantAction(() -> bot.intake.store()))
                .strafeToLinearHeading(shoot.position, shoot.heading.log())
                .stopAndAdd(bot.actionShootThree())

                .build();

        Actions.runBlocking(
                new ParallelAction(
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}
