package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.config.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PIDF;


public class Turret {
    public static Vector2d shooterTransform = new Vector2d(0, 0);
    public static Vector2d aimPoint = new Vector2d(0, 0);

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

    private void aimTowardsTargetPoint() {
        if (MANUAL) {
            targetEncoderPosition = MANUAL_POSITION;
            return;
        }

        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        Vector2d robotPosition = pose.position;
        double robotHeading = pose.heading.log();

        Vector2d shooterFieldPos = robotPosition.plus(new Vector2d(
                shooterTransform.x * Math.cos(robotHeading) - shooterTransform.y * Math.sin(robotHeading),
                shooterTransform.x * Math.sin(robotHeading) + shooterTransform.y * Math.cos(robotHeading)
        ));

        Vector2d delta = aimPoint.minus(shooterFieldPos);
        distanceToGoal = delta.norm();
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
