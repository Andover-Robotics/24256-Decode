package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Bot;

@TeleOp(name = "Reset Turret Encoder")
public class ResetTurretEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        Bot bot = Bot.getInstance(this);

        waitForStart();

        if (isStopRequested()) return;

        bot.turret.resetEncoder();

        sleep(100);
    }
}

