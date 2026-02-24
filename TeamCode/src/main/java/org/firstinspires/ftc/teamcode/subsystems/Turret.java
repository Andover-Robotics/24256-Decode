package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDF;


public class Turret {
    private static Vector2d shooterTransform = new Vector2d(0, 0);
    private static Vector2d redAimPoint = new Vector2d(0, 0);
    private static Vector2d blueAimPoint = new Vector2d(0, 0);

    private MecanumDrive drive;
    private double distanceToGoal;
    private double angleToGoal;
    private double targetEncoderPosition;

    public static double HIGH_LIMIT = 135;
    public static double LOW_LIMIT = -135;

    public static boolean MANUAL = false;
    public static double MANUAL_POSITION = 0;
    
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double G = 386.09;
    public static double DELTA_H = 0;
    public static double HOOD_ANGLE = 0;
    public static boolean VELOCITY_COMPENSATION = false;

    private PIDF controller;

    private double encoderPosition;

    private MotorEx motor;

    private static double ENCODER_TICKS_PER_REV = 360.0 / (141.5 * 104 / 24);

    public Turret(HardwareMap hardwareMap, MecanumDrive drive) {
        this.drive = drive;
        this.motor = new MotorEx(hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
        this.controller = new PIDF(kP, kI, kD, kF, 10, true);
    }

    public void periodic() {
        aimTowardsTargetPoint();
        encoderPosition = motor.getCurrentPosition() * ENCODER_TICKS_PER_REV;

        controller.setGains(kP, kI, kD, kF);
        double output = controller.calculate(targetEncoderPosition, encoderPosition);

        motor.set(output);
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

        Vector2d aimPoint = (Bot.alliance == Bot.Alliance.RED) ? redAimPoint : blueAimPoint;

        PoseVelocity2d velocity = drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        Vector2d robotPosition = pose.position;
        double robotHeading = pose.heading.log();

        Vector2d robotVelocity = Rotation2d.fromDouble(pose.heading.log()).times(velocity.linearVel);
        Vector2d shooterFieldPos = robotPosition.plus(Rotation2d.fromDouble(robotHeading).times(shooterTransform));

        Vector2d delta = aimPoint.minus(shooterFieldPos);
        distanceToGoal = delta.norm();

        if (VELOCITY_COMPENSATION) {
            double time = getTimeToGoal(distanceToGoal);
            delta = delta.minus(robotVelocity.times(time));
        }

        double fieldAngleToGoal = Math.atan2(delta.y, delta.x);

        angleToGoal = normalizeAngle(fieldAngleToGoal - robotHeading);
        double turretHeading = normalizeAngle(-angleToGoal);

        targetEncoderPosition = Math.toDegrees(turretHeading);

        if (targetEncoderPosition > HIGH_LIMIT)
            targetEncoderPosition -= 360;
        else if (targetEncoderPosition < LOW_LIMIT)
            targetEncoderPosition += 360;
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
}
