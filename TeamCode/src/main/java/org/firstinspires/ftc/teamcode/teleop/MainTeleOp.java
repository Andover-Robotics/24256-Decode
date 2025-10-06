package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    private Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(bot.actionPeriodic())
        );

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            double throttle = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            bot.driveRobotCentric(throttle, strafe, turn);

            if (gamepad1.left_bumper) {
                bot.intake.runIntake(Intake.Direction.FORWARD);
            } else if (gamepad1.right_bumper) {
                bot.intake.runIntake(Intake.Direction.REVERSE);
            } else {
                bot.intake.runIntake(Intake.Direction.STOP);
            }

            telemetry.addData("Flywheel Velocity", bot.outtake.getRealVelocity());
        }
    }
}
