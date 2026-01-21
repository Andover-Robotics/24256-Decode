package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.WaitUntilAction;

@Config
public class Bot {
    public enum Alliance {
        RED,
        BLUE
    }

    public static Alliance alliance = Alliance.RED;

    public static Bot instance = null;

    public OpMode opMode;

    // drivetrain motors
    public Motor fl, fr, bl, br;

    public MecanumDrive drive;

    // other subsystems
    public Intake intake;
    public Outtake outtake;

    public static double SHOOT_ONE_DELAY = 0.2;
    public static double SHOOT_THREE_QUICKFIRE_DELAY = 1.25;

    public AprilTag aprilTag;

    private boolean inShootingMode = false;

    public static Pose2d mirror(Pose2d initial) {
        return new Pose2d(new Vector2d(initial.position.x, -initial.position.y), -initial.heading.toDouble());
    }

    public static Vector2d mirror(Vector2d initial) {
        return new Vector2d(initial.x, -initial.y);
    }

    public static Pose2dDual<Arclength> mirror(Pose2dDual<Arclength> pose) {
        return new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());
    }

    private Bot(LinearOpMode opMode) { // new Bot(opMode);
        this.opMode = opMode;
        aprilTag = new AprilTag(opMode);

        intake = new Intake(opMode);
        outtake = new Outtake(opMode, aprilTag);

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

        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));
    }

    public static Bot getInstance(LinearOpMode opMode) {
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

    public void driveRobotCentric(double throttle, double strafe, double turn, double scalar) {
        double[] speeds = {
                (throttle + strafe + turn), // Front Left
                (throttle - strafe - turn), // Front Right
                (throttle - strafe + turn), // Back Left
                (throttle + strafe - turn)  // Back Right
        };
        double maxSpeed = 0;
        for (int i = 0; i < 4; i++) {
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }
        if (maxSpeed > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= maxSpeed;
            }
        }
        for (int i = 0; i < 4; i++) {
            speeds[i] *= scalar;
        }
        fl.set(speeds[0]);
        fr.set(speeds[1]);
        bl.set(speeds[2]);
        br.set(speeds[3]);
    }

    public void periodic() {
        intake.periodic();
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

    public Action actionShootThree() {
        return actionShoot(SHOOT_THREE_QUICKFIRE_DELAY);
    }

    public Action actionShootOne() {
        return actionShoot(SHOOT_ONE_DELAY);
    }

    public Action actionShoot(double time) {
        if (inShootingMode) {
            return new NullAction();
        }
        return new SequentialAction(
                new InstantAction(() -> inShootingMode = true),
                new InstantAction(() -> intake.in()),
                new InstantAction(() -> outtake.enable()),
                new WaitUntilAction(() -> outtake.inTolerance()),
                new InstantAction(() -> intake.openGate()),
                new SleepAction(time),
                new InstantAction(() -> intake.closeGate()),
                new InstantAction(() -> intake.store()),
                new InstantAction(() -> outtake.disable()),
                new InstantAction(() -> inShootingMode = false)
        );
    }
    public boolean inShootingMode() { return inShootingMode; }
}
