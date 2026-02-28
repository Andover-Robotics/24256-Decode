package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.TriggeredTimer;

import java.util.TreeMap;

@Config
public class Outtake {
    private MotorEx motor1;
    private MotorEx motor2;

    private VoltageSensor voltageSensor;

    public static double kP = 0.00065;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.000195;

    private PIDF controller;

    public static double DEFAULT_VELOCITY = 3650;

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

    private static double VELOCITY_TOLERANCE = 100;
    public static double IN_TOLERANCE_TIME = 0.200;

    public static boolean MANUAL = false;
    public static double MANUAL_VELOCITY = 0;

    public static boolean enabled = false;

    private double targetVelocity;
    private double distanceToGoal;
    private double realVelocity;

    private boolean inTolerance;
    private TriggeredTimer inToleranceTimer;

    public Outtake(LinearOpMode opMode) {
        controller = new PIDF(kP, kI, kD, kF);
        motor1 = new MotorEx(opMode.hardwareMap, "outtake1", MotorEx.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2 = new MotorEx(opMode.hardwareMap, "outtake2", MotorEx.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);
        motor2.setInverted(true);

        inToleranceTimer = new TriggeredTimer(IN_TOLERANCE_TIME);

        voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
    }

    public void setDistanceToGoal(double distanceToGoal) {
        this.distanceToGoal = distanceToGoal;
    }

    private double getVelocity() {
        if (MANUAL) {
            return MANUAL_VELOCITY;
        }

        if (VELOCITY_LOOKUP_TABLE.isEmpty()) {
            return 0;
        }

        Double lowerKey = VELOCITY_LOOKUP_TABLE.floorKey(distanceToGoal);
        Double upperKey = VELOCITY_LOOKUP_TABLE.ceilingKey(distanceToGoal);

        if (lowerKey == null || upperKey == null) {
            return 0;
        }

        if (lowerKey.equals(upperKey)) {
            Double velocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
            return velocity != null ? velocity : 0;
        }

        Double lowerVelocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
        Double upperVelocity = VELOCITY_LOOKUP_TABLE.get(upperKey);

        if (lowerVelocity == null || upperVelocity == null) {
            return 0;
        }

        double ratio = (distanceToGoal - lowerKey) / (upperKey - lowerKey);
        return lowerVelocity + ratio * (upperVelocity - lowerVelocity);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getRealVelocity() {
        return realVelocity;
    }

    public void setPower(double power) {
        motor1.set(power);
        motor2.set(power);
    }

    public void periodic() {
        targetVelocity = getVelocity();
        realVelocity = motor1.getVelocity() / 28.0 * 60;

        if (!enabled || targetVelocity == 0) {
            setPower(0);
            return;
        }

        double voltage = voltageSensor.getVoltage();
        controller.setGains(kP, kI, kD, kF / voltage);
        double output = controller.calculate(targetVelocity, realVelocity);

        setPower(output);

        inTolerance = inToleranceTimer.periodic(Math.abs(targetVelocity - realVelocity) < VELOCITY_TOLERANCE);
    }

    public boolean inTolerance() {
        return inTolerance;
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
