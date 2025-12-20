package org.firstinspires.ftc.teamcode.subsystems.shooter;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.config.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Bot;

@Config
public class Outtake {
    private MotorEx motor1;
    private MotorEx motor2;

    // PID constants
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kStatic = 0;
    public static double kV = 0;

    public PIDController controller;

    // flywheel calculations
    private static double FLYWHEEL_GEAR_RATIO = 1.0;
    private static double VELOCITY_TOLERANCE = 50;

    // aim
    public static double shooterA = 0;
    public static double shooterB = 0;
    public static double shooterC = 0;
    public static double shooterD = 0;

    public static boolean MANUAL = true;
    public static double MANUAL_VELOCITY = 4000;

    public static Vector2d offsetFromCenter = new Vector2d(0, 0);

    private boolean enabled = false;

    // TODO: determine goal verticies
    private static Goal redGoal = new Goal(
            new Vector2d(0, 0),
            new Vector2d(0, 0),
            new Vector2d(0, 0)
    );

    private static Goal blueGoal = new Goal(
            new Vector2d(0, 0),
            new Vector2d(0, 0),
            new Vector2d(0, 0)
    );

    private Localizer localizer;

    public double goalDistance;

    private double targetVelocity = 0;

    public Outtake(OpMode opMode, Localizer localizer) {
        controller = new PIDController(kP, kI, kD);
        motor1 = new MotorEx(opMode.hardwareMap, "outtake1", MotorEx.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setInverted(true);
        motor2 = new MotorEx(opMode.hardwareMap, "outtake2", MotorEx.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);

        this.localizer = localizer;
    }

    public Pose2d getPose() {
        Pose2d robotPose = localizer.getPose();

        // rotate offset vector
        double c = Math.cos(robotPose.heading.log());
        double s = Math.sin(robotPose.heading.log());

        return new Pose2d(
                robotPose.position.x * c - robotPose.position.y * s,
                robotPose.position.x * s + robotPose.position.y * c,
                robotPose.heading.log()
        );
    }

    public double getRegressionVelocity() {
        if (MANUAL) return MANUAL_VELOCITY;

        Pose2d startPose = getPose();
        Vector2d hit = ((Bot.alliance == Bot.Alliance.RED) ? redGoal : blueGoal).getGoal(startPose);

        if (hit == null) {
            return 0;
        } else {
            goalDistance = (hit.minus(startPose.position)).norm();
            return shooterA + Math.sqrt(shooterB + goalDistance * shooterC) * shooterD;
        }
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getRealVelocity() {
        return motor1.getVelocity() / motor1.getCPR() / FLYWHEEL_GEAR_RATIO * 60;
    }

    public void setPower(double power) {
        motor1.set(power);
        motor2.set(power);
    }

    public void periodic() {
        targetVelocity = (enabled) ? getRegressionVelocity() : 0;

        double pidOutput = controller.calculate(getRealVelocity(), targetVelocity);
        double ffOutput = kStatic + kV * targetVelocity;

        setPower(pidOutput + ffOutput);
    }

    public boolean inTolerance() {
        return Math.abs(getRealVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public Action actionBlockUntilInTolerance() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                return !inTolerance();
            }
        };
    }
}
