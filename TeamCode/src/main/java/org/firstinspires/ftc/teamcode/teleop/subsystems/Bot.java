package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Bot {
    private static Bot singleton;

    public OpMode opMode;

    // drivetrain motors
    private DcMotor fl, fr, bl, br;

    private Bot(OpMode opMode) {
        fl = opMode.hardwareMap.get(DcMotor.class, "fl");
        fr = opMode.hardwareMap.get(DcMotor.class, "fr");
        bl = opMode.hardwareMap.get(DcMotor.class, "bl");
        br = opMode.hardwareMap.get(DcMotor.class, "br");

        // make sure to set the direction of the motors
    }

    public static Bot getInstance(OpMode opMode) {
        if (singleton == null) {
            singleton = new Bot(opMode);
        }
        singleton.opMode = opMode;
        return singleton;
    }

    public void driveRobotCentric(double throttle, double strafe, double turn) {
        // source: https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        double mag = Math.max(Math.abs(throttle) + Math.abs(strafe) + Math.abs(turn), 1);
        double flPower = (throttle + strafe + turn) / mag;
        double frPower = (throttle - strafe + turn) / mag;
        double blPower = (throttle - strafe - turn) / mag;
        double brPower = (throttle + strafe - turn) / mag;

        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
}
