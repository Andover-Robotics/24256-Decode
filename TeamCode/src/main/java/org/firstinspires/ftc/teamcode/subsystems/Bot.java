package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Bot {
    public static Bot instance;

    public OpMode opMode;

    // drivetrain motors
    private DcMotor fl, fr, bl, br;

    // other subsystems
    public Intake intake;

    private Bot(OpMode opMode) {
        // make sure to set the direction of the motors
        fl = opMode.hardwareMap.get(DcMotor.class, "fl");
        fr = opMode.hardwareMap.get(DcMotor.class, "fr");
        bl = opMode.hardwareMap.get(DcMotor.class, "bl");
        br = opMode.hardwareMap.get(DcMotor.class, "br");

        // initialize other subsystems
        intake = new Intake(opMode.hardwareMap);
    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
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
