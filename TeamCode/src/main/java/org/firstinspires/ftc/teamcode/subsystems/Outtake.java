package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.TriggeredTimer;

import java.util.TreeMap;

@Config
public class Outtake {
    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private double emfResistance;

    private double currentDrawOne;
    private double currentDrawTwo;

    public static double kP = 0.016;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00235;

    private PIDF controller;

    public static double DEFAULT_VELOCITY = 3100;

    private static final TreeMap<Double, Double> VELOCITY_LOOKUP_TABLE = new TreeMap<>();

    static {
        VELOCITY_LOOKUP_TABLE.put(35.0, 2700.0);
        VELOCITY_LOOKUP_TABLE.put(38.0, 2750.0);
        VELOCITY_LOOKUP_TABLE.put(41.0, 2750.0);
        VELOCITY_LOOKUP_TABLE.put(44.0, 2800.0);
        VELOCITY_LOOKUP_TABLE.put(47.0, 2800.0);
        VELOCITY_LOOKUP_TABLE.put(51.0, 2850.0);
        VELOCITY_LOOKUP_TABLE.put(54.0, 2850.0);
        VELOCITY_LOOKUP_TABLE.put(57.0, 2850.0);
        VELOCITY_LOOKUP_TABLE.put(60.0, 2850.0);
        VELOCITY_LOOKUP_TABLE.put(63.0, 2900.0);
        VELOCITY_LOOKUP_TABLE.put(66.0, 2950.0);
        VELOCITY_LOOKUP_TABLE.put(69.0, 2950.0);
        VELOCITY_LOOKUP_TABLE.put(72.0, 3050.0);
        VELOCITY_LOOKUP_TABLE.put(75.0, 3150.0);
        VELOCITY_LOOKUP_TABLE.put(78.0, 3200.0);
        VELOCITY_LOOKUP_TABLE.put(81.0, 3250.0);
        VELOCITY_LOOKUP_TABLE.put(84.0, 3300.0);
        VELOCITY_LOOKUP_TABLE.put(87.0, 3350.0);
    }

    public static double VELOCITY_TOLERANCE = 100;
    public static double IN_TOLERANCE_TIME = 0.150;
    public static double WINDUP_RANGE = 100;

    public static boolean MANUAL = false;
    public static double MANUAL_VELOCITY = 0;

    private boolean enabled = false;

    private double targetVelocity;
    private double distanceToGoal;
    private double realVelocity;

    private boolean inTolerance;
    private TriggeredTimer inToleranceTimer;

    private boolean usingPrimaryEncoder = true;
    private boolean shooterMotorDisconnected = false;

    private static double ENCODER_REV_PER_TICK = 1 / 28.0 * 60;

    public Outtake(LinearOpMode opMode) {
        controller = new PIDF(kP, kI, kD, kF, WINDUP_RANGE, false);
        motor1 = opMode.hardwareMap.get(DcMotorEx.class, "outtake1");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2 = opMode.hardwareMap.get(DcMotorEx.class, "outtake2");
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        inToleranceTimer = new TriggeredTimer(IN_TOLERANCE_TIME);
    }

    public void setDistanceToGoal(double distanceToGoal) {
        this.distanceToGoal = distanceToGoal;
    }

    private double getVelocity() {
        if (MANUAL) {
            return MANUAL_VELOCITY;
        }

        Double lowerKey = VELOCITY_LOOKUP_TABLE.floorKey(distanceToGoal);
        Double upperKey = VELOCITY_LOOKUP_TABLE.ceilingKey(distanceToGoal);

        if (lowerKey == null || upperKey == null) {
            return DEFAULT_VELOCITY;
        }

        if (lowerKey.equals(upperKey)) {
            Double velocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
            return velocity != null ? velocity : DEFAULT_VELOCITY;
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
        motor1.setPower(power);
        motor2.setPower(power * -1.0);
    }

    public void updateMotorData() {
        if (usingPrimaryEncoder) {
            realVelocity = motor2.getVelocity() * ENCODER_REV_PER_TICK * -1.0;
        } else {
            realVelocity = motor1.getVelocity() * ENCODER_REV_PER_TICK;
        }

        if (Math.abs(targetVelocity) < 0.001)
            return;

        currentDrawOne = motor1.getCurrent(CurrentUnit.MILLIAMPS);
        currentDrawTwo = motor2.getCurrent(CurrentUnit.MILLIAMPS) * -1.0;

        if (enabled && (Math.abs(currentDrawOne) < 0.001 || Math.abs(currentDrawTwo) < 0.001))
            shooterMotorDisconnected = true;

        if (enabled && Math.abs(realVelocity) < 0.001)
            usingPrimaryEncoder = !usingPrimaryEncoder;
    }

    public boolean isRunning() {
        return enabled && targetVelocity != 0;
    }

    public void periodic() {
        updateMotorData();
        targetVelocity = getVelocity();
        inTolerance = inToleranceTimer.periodic(Math.abs(targetVelocity - realVelocity) < VELOCITY_TOLERANCE);

        if (!isRunning()) {
            setPower(0);
            return;
        }

        double voltage = Bot.getInstance().getBatteryVoltage();
        controller.setGains(kP, kI, kD, kF);
        double output = controller.calculate(targetVelocity, realVelocity);

        setPower(output / voltage);

        emfResistance = voltage / (currentDrawOne + currentDrawTwo);
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

    public boolean isShooterMotorDisconnected() {
        return shooterMotorDisconnected;
    }

    public boolean isUsingPrimaryEncoder() {
        return usingPrimaryEncoder;
    }

    public double getEMFResistance() {
        return emfResistance;
    }
}
