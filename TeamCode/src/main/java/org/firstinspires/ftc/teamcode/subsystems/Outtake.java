package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.TriggeredTimer;

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

    private static double VELOCITY_TOLERANCE = 100;
    public static double IN_TOLERANCE_TIME = 0.200;

    public static boolean MANUAL = false;
    public static double MANUAL_VELOCITY = 0;

    public static boolean enabled = false;

    private double targetVelocity = 0;
    private double distanceToGoal;

    private boolean inTolerance;
    private TriggeredTimer inToleranceTimer;

    public Outtake(LinearOpMode opMode) {
        controller = new PIDController(kP, kI, kD);
        motor1 = new MotorEx(opMode.hardwareMap, "outtake1", MotorEx.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2 = new MotorEx(opMode.hardwareMap, "outtake2", MotorEx.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);
        motor2.setInverted(true);

        inToleranceTimer = new TriggeredTimer(IN_TOLERANCE_TIME);
    }

    public void setDistanceToGoal(double distanceToGoal) {
        this.distanceToGoal = distanceToGoal;
    }

    private double getRegressionFallback(double distance) {
        return SHOOTER_A * distanceToGoal * distance + SHOOTER_B * distanceToGoal + SHOOTER_C;
    }

    private double getVelocity() {
        if (MANUAL) {
            return MANUAL_VELOCITY;
        }

        if (VELOCITY_LOOKUP_TABLE.isEmpty()) {
            return getRegressionFallback(distanceToGoal);
        }

        Double lowerKey = VELOCITY_LOOKUP_TABLE.floorKey(distanceToGoal);
        Double upperKey = VELOCITY_LOOKUP_TABLE.ceilingKey(distanceToGoal);

        if (lowerKey == null || upperKey == null) {
            return getRegressionFallback(distanceToGoal);
        }

        if (lowerKey.equals(upperKey)) {
            Double velocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
            return velocity != null ? velocity : getRegressionFallback(distanceToGoal);
        }

        Double lowerVelocity = VELOCITY_LOOKUP_TABLE.get(lowerKey);
        Double upperVelocity = VELOCITY_LOOKUP_TABLE.get(upperKey);

        if (lowerVelocity == null || upperVelocity == null) {
            return getRegressionFallback(distanceToGoal);
        }

        double ratio = (distanceToGoal - lowerKey) / (upperKey - lowerKey);
        return lowerVelocity + ratio * (upperVelocity - lowerVelocity);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getRealVelocity() {
        return motor1.getVelocity() / 28.0 * 60;
    }

    public void setPower(double power) {
        motor1.set(power);
        motor2.set(power);
    }

    public void periodic() {
        targetVelocity = getVelocity();

        if (!enabled || targetVelocity == 0) {
            setPower(0);
            return;
        }

        controller.setPID(kP, kI, kD);

        double pidOutput = controller.calculate(getRealVelocity(), targetVelocity);
        double ffOutput = kStatic + kV * targetVelocity;

        setPower(pidOutput + ffOutput);

        inTolerance = inToleranceTimer.periodic(Math.abs(targetVelocity - getRealVelocity()) < VELOCITY_TOLERANCE);
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
