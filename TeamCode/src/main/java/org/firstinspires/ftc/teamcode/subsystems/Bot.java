package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Outtake;

public class Bot {
    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance;

    public static Bot instance = null;

    public OpMode opMode;

    // drivetrain motors
    public Motor fl, fr, bl, br;

    // other subsystems
    public Intake intake;
    public Outtake outtake;

    MecanumDrive drive;

    public static double SHOOT_THREE_DELAY = 0.0;
    public static double SHOOT_ONE_DELAY = 0.0;

    public static Pose2d mirror(Pose2d initial) {
        return new Pose2d(new Vector2d(initial.position.x, -initial.position.y), -initial.heading.log());
    }

    public static Pose2d redResetPose = new Pose2d(0, 0, Math.toRadians(180));
    public static Pose2d blueResetPose = mirror(redResetPose);

    private Bot(OpMode opMode) { // new Bot(opMode);
        // make sure to set the direction of the motors
        fl = new Motor(opMode.hardwareMap, "fl");
        fr = new Motor(opMode.hardwareMap, "fr");
        bl = new Motor(opMode.hardwareMap, "bl");
        br = new Motor(opMode.hardwareMap, "br");
        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        bl.setRunMode(Motor.RunMode.RawPower);
        br.setRunMode(Motor.RunMode.RawPower);
        fl.setInverted(true);
        bl.setInverted(true);

        drive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));

        intake = new Intake(opMode);
        outtake = new Outtake(opMode, null);
    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    public static void switchAlliance() {
        if (alliance == Alliance.RED) {
            alliance = Alliance.BLUE;
        } else {
            alliance = Alliance.RED;
        }
    }

    public void setPose(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    public Pose2d getPose() {
        return drive.localizer.getPose();
    }

    public void driveRobotCentric(double throttle, double strafe, double turn) {
        double mag = Math.max(Math.abs(throttle) + Math.abs(strafe) + Math.abs(turn), 1);

        double flPower = (throttle + strafe + turn) / mag;
        double frPower = (throttle - strafe - turn) / mag;
        double blPower = (throttle - strafe + turn) / mag;
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

    public Action actionShoot(double delay) {
        return new SequentialAction(
                new InstantAction(() -> intake.in()),
                new InstantAction(() -> outtake.enable()),
                outtake.actionBlockUntilInTolerance(),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(delay),
                new InstantAction(() -> intake.closeGate()),
                new InstantAction(() -> intake.store()),
                new InstantAction(() -> outtake.disable())
        );
    }
}
