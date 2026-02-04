package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.config.Localizer;


public class Turret {
    public static Vector2d shooterTransform = new Vector2d(0, 0);
    public static Vector2d aimPoint = new Vector2d(0, 0);

    private Localizer localizer;
    private double distanceToGoal;
    private double angleToGoal;
    private double targetEncoderPosition;

    public static double HIGH_LIMIT = 135;
    public static double LOW_LIMIT = -135;
    
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    
    private PIDFController controller;

    private double encoderPosition;

    private MotorEx motor;

    private static double ENCODER_TICKS_PER_REV = 360.0 / (141.5 * 104 / 24);

    public Turret(HardwareMap hardwareMap, Localizer localizer) {
        this.localizer = localizer;
        this.motor = new MotorEx(hardwareMap, "turret", Motor.GoBILDA.RPM_1150);
    }

    public void periodic() {
        aimTowardsTargetPoint();
        updateEncoderPosition();

        controller.setPIDF(kP, kI, kD, kF);
        controller.setSetPoint(targetEncoderPosition);
        double p = controller.calculate(encoderPosition);

        motor.set(p);
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getAngleToGoal() {
        return angleToGoal;
    }

    private static double normalizeAngle(double angle) {
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        while (angle > Math.PI) angle -= 2 * Math.PI;
        return angle;
    }

    public void aimTowardsTargetPoint() {
        Pose2d pose = localizer.getPose();

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
        double turretHeading = normalizeAngle(-angleToGoal + robotHeading);

        targetEncoderPosition = Math.toDegrees(turretHeading);

        if (targetEncoderPosition > HIGH_LIMIT) targetEncoderPosition -= 360;
        if (targetEncoderPosition < LOW_LIMIT) targetEncoderPosition += 360;
    }

    public void updateEncoderPosition() {
        encoderPosition = motor.getCurrentPosition() * ENCODER_TICKS_PER_REV;
    }
}
