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

@Autonomous(name = "Decode_6 Ball_No Gate_Far Side_Auto")

public class SixBall_NoGate_FarSide extends LinearOpMode {
    //NOT ACTUAL VALUES; PLACEHOLDERS FOR NOW. I don't know the strat for far side
    public static Pose2d redAllianceStartPose = new Pose2d();
    public static Pose2d shoot = new Pose2d();

    public static Vector2d preFirstIntake = new Vector2d();

    public static Vector2d firstIntake = new Vector2d();
    public static Vector2d preSecondIntake = new Vector2d();
    public static Vector2d secondIntake = new Vector2d();

    public static Vector2d preThirdIntake = new Vector2d();
    public static Vector2d thirdIntake = new Vector2d();

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
                //Shooting code; make later

                .build();

        Actions.runBlocking(
                new ParallelAction( /* ends when auto ends */
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}
