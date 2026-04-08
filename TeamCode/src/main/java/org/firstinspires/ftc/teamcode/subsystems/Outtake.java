package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.TriggeredTimer;

import java.util.TreeMap;

@Config
public class Outtake {
    private MotorEx motor1;
    private MotorEx motor2;

    public static double kP = 0.011;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00255;

    private PIDF controller;

    public static double DEFAULT_VELOCITY = 3100;

    private static final TreeMap<Double, Double> VELOCITY_LOOKUP_TABLE = new TreeMap<>();

    static {
        VELOCITY_LOOKUP_TABLE.put(46.0, 3050.0);
        VELOCITY_LOOKUP_TABLE.put(50.0, 3100.0);
        VELOCITY_LOOKUP_TABLE.put(54.0, 3150.0);
        VELOCITY_LOOKUP_TABLE.put(59.0, 3200.0);
        VELOCITY_LOOKUP_TABLE.put(64.0, 3350.0);
        VELOCITY_LOOKUP_TABLE.put(68.0, 3400.0);
        VELOCITY_LOOKUP_TABLE.put(72.0, 3400.0);
        VELOCITY_LOOKUP_TABLE.put(76.0, 3550.0);
        VELOCITY_LOOKUP_TABLE.put(80.0, 3650.0);
        VELOCITY_LOOKUP_TABLE.put(84.0, 3750.0);
    }

    public static double VELOCITY_TOLERANCE = 150;
    public static double IN_TOLERANCE_TIME = 0.300;

    public static boolean MANUAL = false;
    public static double MANUAL_VELOCITY = 0;

    private boolean enabled = false;

    private double targetVelocity;
    private double distanceToGoal;
    private double realVelocity;

    private boolean inTolerance;
    private TriggeredTimer inToleranceTimer;

    public Outtake(LinearOpMode opMode) {
        controller = new PIDF(kP, kI, kD, kF);
        motor1 = new MotorEx(opMode.hardwareMap, "outtake1", MotorEx.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setInverted(true);
        motor2 = new MotorEx(opMode.hardwareMap, "outtake2", MotorEx.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);

        inToleranceTimer = new TriggeredTimer(IN_TOLERANCE_TIME);
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
            return DEFAULT_VELOCITY;
        }

        if (lowerKey.equals(upperKey)) {
            Double velocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
            return velocity != null ? velocity : 0;
        }

        Double lowerVelocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
        Double upperVelocity = VELOCITY_LOOKUP_TABLE.get(upperKey);

        if (lowerVelocity == null || upperVelocity == null) {
            return DEFAULT_VELOCITY;
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
        inTolerance = inToleranceTimer.periodic(Math.abs(targetVelocity - realVelocity) < VELOCITY_TOLERANCE);

        if (!enabled || targetVelocity == 0) {
            setPower(0);
            return;
        }

        double voltage = Bot.getInstance().getBatteryVoltage();
        controller.setGains(kP, kI, kD, kF);
        double output = controller.calculate(targetVelocity, realVelocity);

        setPower(output / voltage);
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
