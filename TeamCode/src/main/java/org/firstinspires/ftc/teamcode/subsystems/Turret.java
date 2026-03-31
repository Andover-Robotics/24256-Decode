package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDF;

@Config
public class Turret {
    public static double SHOOTER_X = 68;
    public static double SHOOTER_Y = -68;
    private static Vector2d shooterTransform = new Vector2d(-1.65, 0);
    private static Vector2d redAimPoint = new Vector2d(69, -69);
    private static Vector2d blueAimPoint = Bot.mirror(redAimPoint);

    private MecanumDrive drive;
    private double distanceToGoal;
    private double angleToGoal;
    private double targetEncoderPosition;

    public static double HIGH_LIMIT = Math.toRadians(45);
    public static double LOW_LIMIT = Math.toRadians(-150);

    public static boolean MANUAL = false;
    public static double MANUAL_POSITION = 0;

    public static double kP = 0.41;
    public static double kI = 0.80;
    public static double kD = 0.017;
    public static double kF = 0;
    public static double windupRange = 3;
    private double adjustable = 0;

    public static double G = 386.09;
    public static double DELTA_H = 38.5;
    public static double HOOD_ANGLE = Math.toRadians(90 - 41);
    public static boolean VELOCITY_COMPENSATION = true;

    private PIDF controller;

    private double encoderPosition;

    private MotorEx motor;

    private static double ENCODER_DEGREES_PER_TICK = 360.0 / (145.1 * 104 / 14);

    public Turret(LinearOpMode opMode, MecanumDrive drive) {
        this.drive = drive;
        motor = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        controller = new PIDF(kP, kI, kD, kF, windupRange, true);
    }

    public void periodic() {
        redAimPoint = new Vector2d(SHOOTER_X, SHOOTER_Y);
        blueAimPoint = Bot.mirror(redAimPoint);
        aimTowardsTargetPoint();
        encoderPosition = motor.getCurrentPosition() * ENCODER_DEGREES_PER_TICK;

        double voltage = Bot.getInstance().getBatteryVoltage();
        controller.setGains(kP, kI, kD, kF);
        double output = controller.calculate(targetEncoderPosition, encoderPosition);

        motor.set(output / voltage);
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    private static double getTimeToGoal(double deltaX) {
        double tSquared = 2 * (deltaX * Math.tan(HOOD_ANGLE) - DELTA_H) / G;
        if (tSquared < 0)
            return 0;
        return Math.sqrt(tSquared);
    }

    private void aimTowardsTargetPoint() {
        if (MANUAL) {
            targetEncoderPosition = MANUAL_POSITION;
            return;
        }

        PoseVelocity2d velocity = Bot.getInstance().getRobotVelocity();
        Vector2d aimPoint = (Bot.alliance == Bot.Alliance.RED) ? redAimPoint : blueAimPoint;
        Pose2d pose = drive.localizer.getPose();

        Vector2d robotPosition = pose.position;
        double robotHeading = pose.heading.log();

        Vector2d robotVelocity = Rotation2d.fromDouble(-pose.heading.log()).times(velocity.linearVel);
        Vector2d shooterFieldPos = robotPosition.plus(Rotation2d.fromDouble(robotHeading).times(shooterTransform));

        Vector2d delta = aimPoint.minus(shooterFieldPos);

        if (VELOCITY_COMPENSATION) {
            Vector2d compensatedDelta = delta;
            for (int i = 0; i < 3; i++) {
                double time = getTimeToGoal(compensatedDelta.norm());
                compensatedDelta = delta.minus(robotVelocity.times(time));
            }
            delta = compensatedDelta;
        }

        double fieldAngleToGoal = delta.angleCast().log();
        distanceToGoal = delta.norm();

        angleToGoal = normalizeAngle(fieldAngleToGoal - robotHeading);

        targetEncoderPosition = normalizeAngle(-angleToGoal + Math.toRadians(adjustable));

        while (targetEncoderPosition > HIGH_LIMIT)
            targetEncoderPosition -= 2 * Math.PI;
        while (targetEncoderPosition < LOW_LIMIT)
            targetEncoderPosition += 2 * Math.PI;

        targetEncoderPosition = Math.toDegrees(Math.max(LOW_LIMIT, Math.min(HIGH_LIMIT, targetEncoderPosition)));
    }

    public double getTargetEncoderPosition() {
        return targetEncoderPosition;
    }

    public double getEncoderPosition() {
        return encoderPosition;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getAngleToGoal() {
        return angleToGoal;
    }

    public void resetEncoder() {
        motor.stopAndResetEncoder();
    }

    public double getAdjustable() {
        return adjustable;
    }

    public void setAdjustable(double adjustable) {
        this.adjustable = adjustable;
    }
}
