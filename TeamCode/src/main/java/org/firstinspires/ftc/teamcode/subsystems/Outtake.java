package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.util.TriggeredTimer;

import java.util.TreeMap;

@Config
public class Outtake {
    private DcMotorEx motor1;
    private DcMotorEx motor2;

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

    private boolean usingPrimaryEncoder = true;
    private boolean shooterMotorDisconnected = false;

    private static double ENCODER_REV_PER_TICK = 1 / 28.0 * 60;

    public Outtake(LinearOpMode opMode) {
        controller = new PIDF(kP, kI, kD, kF);
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

        double currentDrawOne = motor1.getCurrent(CurrentUnit.MILLIAMPS);
        double currentDrawTwo = motor2.getCurrent(CurrentUnit.MILLIAMPS) * -1.0;

        if (Math.abs(currentDrawOne) < 0.001 || Math.abs(currentDrawTwo) < 0.001)
            shooterMotorDisconnected = true;

        if (Math.abs(realVelocity) < 0.001)
            usingPrimaryEncoder = !usingPrimaryEncoder;
    }

    public void periodic() {
        updateMotorData();
        targetVelocity = getVelocity();
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

    public boolean isShooterMotorDisconnected() {
        return shooterMotorDisconnected;
    }

    public boolean isUsingPrimaryEncoder() {
        return usingPrimaryEncoder;
    }
}
