package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
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
    public static Pose2d redAllianceStartPose = new Pose2d(0, 0, 0);


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
                .waitSeconds(3)
                .build();

        drive.localizer.setPose(startPose);
        Actions.runBlocking(
                new RaceAction( /* ends when auto ends */
                        bot.actionPeriodic(),
                        auto
                )
        );
    }
}
