package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.TreeMap;

@Config
public class Outtake {
    private MotorEx motor1;
    private MotorEx motor2;

    public static double kP = 0.00065;
    public static double kI = 0;
    public static double kD = 0;
    public static double kStatic = 0;
    public static double kV = 0.000195;

    public PIDController controller;

    private static double FLYWHEEL_GEAR_RATIO = 1.0;
    private static double VELOCITY_TOLERANCE = 150;

    public static double DEFAULT_VELOCITY = 3600;

    private static final TreeMap<Double, Double> VELOCITY_LOOKUP_TABLE = new TreeMap<>();

    static {
        VELOCITY_LOOKUP_TABLE.put(25.0, 3550.0);
        VELOCITY_LOOKUP_TABLE.put(28.0, 3550.0);
        VELOCITY_LOOKUP_TABLE.put(31.0, 3550.0);
        VELOCITY_LOOKUP_TABLE.put(34.0, 3600.0);
        VELOCITY_LOOKUP_TABLE.put(37.0, 3600.0);
        VELOCITY_LOOKUP_TABLE.put(40.0, 3650.0);
        VELOCITY_LOOKUP_TABLE.put(43.0, 3800.0);
        VELOCITY_LOOKUP_TABLE.put(46.0, 3900.0);
        VELOCITY_LOOKUP_TABLE.put(49.0, 3900.0);
        VELOCITY_LOOKUP_TABLE.put(52.0, 3950.0);
        VELOCITY_LOOKUP_TABLE.put(55.0, 3950.0);
        VELOCITY_LOOKUP_TABLE.put(58.0, 4000.0);
        VELOCITY_LOOKUP_TABLE.put(61.0, 4050.0);
    }

    public static double SHOOTER_A = 0.0638251;
    public static double SHOOTER_B = 10.53669;
    public static double SHOOTER_C = 3193.94494;

    public static double IN_TOLERANCE_TIME = 0.400 * 1000;

    public static boolean MANUAL = false;
    public static double MANUAL_VELOCITY = 0;

    public static boolean enabled = false;

    public static double ATAG_HEIGHT = 29.5;
    public static double CAMERA_HEIGHT = 13.5;

    public Double hitDistance = null;

    private double targetVelocity = 0;

    private AprilTag aprilTag;

    public double bearing;
    public double yaw;

    private double beginTs = -1.0;

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
        if (aprilTag == null || aprilTag.goal == null) {
            return null;
        }

        AprilTag.AprilTagResult goal = aprilTag.goal;
        if (goal.ftcPose == null) {
            return null;
        }

        double ftcPoseRange = goal.ftcPose.range;
        bearing = Math.toRadians(goal.ftcPose.bearing);

        double heightDifference = ATAG_HEIGHT - CAMERA_HEIGHT;
        double atagDistance = Math.sqrt(ftcPoseRange * ftcPoseRange - heightDifference * heightDifference);

        return atagDistance * Math.cos(bearing);
    }

    private double getRegressionFallback(double distance) {
        return SHOOTER_A * distance * distance + SHOOTER_B * distance + SHOOTER_C;
    }

    private double interpolateVelocity(double distance) {
        if (VELOCITY_LOOKUP_TABLE.isEmpty()) {
            return getRegressionFallback(distance);
        }

        Double lowerKey = VELOCITY_LOOKUP_TABLE.floorKey(distance);
        Double upperKey = VELOCITY_LOOKUP_TABLE.ceilingKey(distance);

        if (lowerKey == null || upperKey == null) {
            return getRegressionFallback(distance);
        }

        if (lowerKey.equals(upperKey)) {
            Double velocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
            return velocity != null ? velocity : getRegressionFallback(distance);
        }

        Double lowerVelocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
        Double upperVelocity = VELOCITY_LOOKUP_TABLE.get(upperKey);

        if (lowerVelocity == null || upperVelocity == null) {
            return getRegressionFallback(distance);
        }

        double ratio = (distance - lowerKey) / (upperKey - lowerKey);
        return lowerVelocity + ratio * (upperVelocity - lowerVelocity);
    }

    public double getRegressionVelocity() {
        if (MANUAL) {
            return MANUAL_VELOCITY;
        }

        if (hitDistance == null) {
            if (targetVelocity == 0) {
                return DEFAULT_VELOCITY;
            } else {
                return targetVelocity;
            }
        } else {
            return interpolateVelocity(hitDistance);
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
        if (Math.abs(getRealVelocity() - targetVelocity) < VELOCITY_TOLERANCE) {
            if (beginTs < 0) {
                beginTs = System.currentTimeMillis();
            }
            double deltaTime = System.currentTimeMillis() - beginTs;
            return deltaTime > IN_TOLERANCE_TIME;
        } else {
            beginTs = -1;
        }
        return false;
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
}
