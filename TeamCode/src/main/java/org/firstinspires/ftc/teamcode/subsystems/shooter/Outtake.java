package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.config.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Bot;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

@Config
public class Outtake {
    private MotorEx motor1;
    private MotorEx motor2;

    // PID constants
    public static double kP = 0.00025;
    public static double kI = 0;
    public static double kD = 0;
    public static double kStatic = 0;
    public static double kV = 0.000185;

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

    private static List<Vector2d> redGoalCorners = Arrays.asList(
            new Vector2d(70.06, -47.65),
            new Vector2d(48.41, -63.36),
            new Vector2d(48.23, -69.94),
            new Vector2d(70.16, -70.20)
    );

    private static Goal redGoal = new Goal(redGoalCorners);

    private static Goal blueGoal = new Goal(redGoalCorners.stream().map(Bot::mirror).collect(Collectors.toList()));

    public Vector2d hit = null;
    public Double hitDistance = null;

    private Localizer localizer;

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
                robotPose.position.x + offsetFromCenter.x * c - offsetFromCenter.y * s,
                robotPose.position.y + offsetFromCenter.x * s + offsetFromCenter.y * c,
                robotPose.heading.log()
        );
    }

    public void getHit() {
        if (localizer == null) return;
        Pose2d startPose = getPose();
        hit = ((Bot.alliance == Bot.Alliance.RED) ? redGoal : blueGoal).getGoal(startPose);
        if (hit == null) {
            hitDistance = null;
            return;
        }
        hitDistance = (hit.minus(startPose.position)).norm();
    }

    public double getRegressionVelocity() {
        if (MANUAL) return MANUAL_VELOCITY;

        if (hitDistance == null) {
            return 0;
        } else {
            return shooterA + Math.sqrt(shooterB + hitDistance * shooterC) * shooterD;
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
        getHit();

        controller.setPID(kP, kI, kD);
        targetVelocity = getRegressionVelocity();

        if (!enabled || targetVelocity == 0) {
            setPower(0);
            return;
        }

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
        controller.reset();
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }
}
