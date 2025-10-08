package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Bot {
    public static Bot instance;

    public OpMode opMode;

    // drivetrain motors
    private Motor fl, fr, bl, br;

    // other subsystems
    public Intake intake;
    public Outtake outtake;

    private Bot(OpMode opMode) {
        // make sure to set the direction of the motors
        HardwareMap hardwareMap = opMode.hardwareMap;
        fl = new Motor(hardwareMap, "fl");
        fr = new Motor(hardwareMap, "fr");
        bl = new Motor(hardwareMap, "bl");
        br = new Motor(hardwareMap, "br");
        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);

        // initialize other subsystems
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
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

        fl.set(flPower);
        fr.set(frPower);
        bl.set(blPower);
        br.set(brPower);
    }

    public void periodic() {
        outtake.periodic();
    }

    public Action actionPeriodic() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                periodic();
                return true;
            }
        };
    }
}
