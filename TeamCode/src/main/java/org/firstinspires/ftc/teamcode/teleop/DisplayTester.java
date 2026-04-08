package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Display;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Display Tester")
public class DisplayTester extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Display display = new Display(this);

        int i = 0;

        display.periodic(Intake.PossessionState.NONE);
        sleep(250);
        display.periodic(Intake.PossessionState.ONE);
        sleep(250);
        display.periodic(Intake.PossessionState.TWO);
        sleep(250);
        display.periodic(Intake.PossessionState.THREE);
        sleep(250);
        display.periodic(Intake.PossessionState.OVER);
        sleep(250);
        display.periodic(Intake.PossessionState.NONE);
    }
}