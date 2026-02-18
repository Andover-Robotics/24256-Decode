package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Display;

@TeleOp(name = "Screen Tester")
public class DisplayTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Display display = new Display(this);

        int i = 0;

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.xWasPressed()) {
                i = (i + 1) % 4;
            }
            display.periodic(i > 0, i > 1, i > 2);
        }
    }
}
