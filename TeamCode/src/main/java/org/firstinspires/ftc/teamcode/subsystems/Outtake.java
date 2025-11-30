package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Outtake {
    private MotorEx motor1;
    private MotorEx motor2;

    // PID constants
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kStatic = 0;
    public static double kV = 0;

    public PIDController controller;

    // flywheel calculations
    private static double FLYWHEEL_GEAR_RATIO = 1.0;
    private static double VELOCITY_TOLERANCE = 50;

    // aim
    public AprilTag aprilTag;
    public static double shooterA = 0;
    public static double shooterB = 0;
    public static double shooterC = 0;
    public static double shooterD = 0;

    public static boolean MANUAL = true;
    public static double MANUAL_VELOCITY = 4000;


    private double targetVelocity = 0;

    public Outtake(OpMode opMode) {
        controller = new PIDController(kP, kI, kD);
        motor1 = new MotorEx(opMode.hardwareMap, "outtake1", MotorEx.GoBILDA.BARE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor1.setInverted(true);
        motor2 = new MotorEx(opMode.hardwareMap, "outtake2", MotorEx.GoBILDA.BARE);
        motor2.setRunMode(Motor.RunMode.RawPower);

        if (MANUAL) {
            aprilTag = null;
        } else {
            aprilTag = new AprilTag(opMode.hardwareMap);
        }
    }

    public double getRegressionVelocity() {
        Double distance = aprilTag.getDistance();

        if (aprilTag == null || distance == null) {
            return MANUAL_VELOCITY;
        } else {
            this.aprilTag.updateDetections();
            return shooterA * Math.sqrt(shooterB * distance + shooterC) + shooterD;
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
        targetVelocity = getRegressionVelocity();

        double pidOutput = controller.calculate(getRealVelocity(), targetVelocity);
        double ffOutput = kStatic + kV * targetVelocity;

        setPower(pidOutput + ffOutput);
    }

    public boolean inTolerance() {
        return Math.abs(getRealVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }
}
