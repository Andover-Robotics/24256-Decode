package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class Outtake {
    private MotorEx motor;

    // PID constants
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kStatic = 0;
    public static double kV = 0;

    public PIDController controller;

    // flywheel calculations
    private static double FLYWHEEL_GEAR_RATIO = 1.0;
    private static double VELOCITY_BOUND = 50;

    private double targetVelocity = 0;

    public Outtake(OpMode opMode) {
        controller = new PIDController(kP, kI, kD);
        motor = new MotorEx(opMode.hardwareMap, "outtake", MotorEx.GoBILDA.RPM_312);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setInverted(true);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getRealVelocity() {
        return motor.getVelocity() / motor.getCPR() / FLYWHEEL_GEAR_RATIO * 60;
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void periodic() {
        double pidOutput = controller.calculate(getRealVelocity(), targetVelocity);
        double ffOutput = kStatic + kV * targetVelocity;

        setPower(pidOutput + ffOutput);
    }

    public boolean inTolerance() {
        return Math.abs(getRealVelocity() - targetVelocity) < VELOCITY_BOUND;
    }

}
