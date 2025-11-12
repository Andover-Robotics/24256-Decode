package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Decode TeleOp")
public class MainTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        Bot bot = Bot.getInstance(this);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            double throttle = gp1.getLeftY();
            double strafe = gp1.getLeftX();
            double turn = gp1.getRightX();

            bot.driveRobotCentric(throttle, strafe, turn);
            bot.periodic();

            gp1.readButtons();
            if (gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2) {
                bot.intake.in();
            } else if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2) {
                bot.intake.out();
            } else {
                bot.intake.store();
            }

            if (gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.intake.openGate();
            } else if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.intake.closeGate();
            }

//            telemetry.addData("Flywheel Velocity", bot.outtake.getRealVelocity());
            telemetry.update();
        }
    }
}
