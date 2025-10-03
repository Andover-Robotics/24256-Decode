package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@TeleOp(name = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    private Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double throttle = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            bot.driveRobotCentric(throttle, strafe, turn);
        }
    }
}
