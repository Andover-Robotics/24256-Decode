package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class Outtake {
    private MotorEx motor1;
    private MotorEx motor2;

    // PID constants
    public static double kP = 0.0004;
    public static double kI = 0;
    public static double kD = 0;
    public static double kStatic = 0;
    public static double kV = 0.000185;

    public PIDController controller;

    // flywheel calculations
    private static double FLYWHEEL_GEAR_RATIO = 1.0;
    private static double VELOCITY_TOLERANCE = 50;

    // aim
    public static double shooterA = 0.248463;
    public static double shooterB = -9.02865;
    public static double shooterC = 3535.63289;

    public static boolean MANUAL = false;
    public static double MANUAL_VELOCITY = 0;
    public static double DEFAULT_VELOCITY = 3900;

    public static boolean enabled = false;

    public static double HEIGHT_FROM_CAM_TO_ATAG = 15;
    public static double TO_INSIDE_OFFSET = 0;

    public Double hitDistance = null;

    private double targetVelocity = 0;

    private AprilTag aprilTag;

    public double bearing;

    public Outtake(LinearOpMode opMode, AprilTag aprilTag) {
        controller = new PIDController(kP, kI, kD);
        motor1 = new MotorEx(opMode.hardwareMap, "outtake1", MotorEx.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2 = new MotorEx(opMode.hardwareMap, "outtake2", MotorEx.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);
        motor2.setInverted(true);

        this.aprilTag = aprilTag;
    }

    private Double getHitDistance() {
        AprilTag.AprilTagResult goal = aprilTag.goal;
        if (goal == null)
            return null;

        bearing = Math.toRadians(goal.ftcPose.bearing);
        double directDistance = goal.ftcPose.range;

        double twoDimDistance = Math.sqrt(
                directDistance * directDistance
                        - HEIGHT_FROM_CAM_TO_ATAG * HEIGHT_FROM_CAM_TO_ATAG
        );
        double bearingCorrection = twoDimDistance * Math.cos(bearing);

        return bearingCorrection + TO_INSIDE_OFFSET;
    }

    public double getRegressionVelocity() {
        if (MANUAL) return MANUAL_VELOCITY;

        if (hitDistance == null) {
            return DEFAULT_VELOCITY;
        } else {
            return shooterA * Math.pow(hitDistance, 2) + shooterB * hitDistance + shooterC;
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
        aprilTag.updateDetections();
        hitDistance = getHitDistance();
        targetVelocity = getRegressionVelocity();

        if (!enabled || targetVelocity == 0) {
            setPower(0);
            return;
        }

        controller.setPID(kP, kI, kD);

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
