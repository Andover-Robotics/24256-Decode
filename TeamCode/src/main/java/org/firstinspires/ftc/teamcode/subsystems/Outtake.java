package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Outtake {
    private MotorEx motor;

    // PID constants
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public PIDFController controller;

    // flywheel calculations
    private static double FLYWHEEL_VELOCITY_SCALING_FACTOR = 1.0;
    private static double VELOCITY_BOUND = 50;

    private double targetVelocity = 0;

    public Outtake(HardwareMap hardwareMap) {
        controller.setPIDF(kP, kI, kD, kF);
        controller.setTolerance(VELOCITY_BOUND);
        motor = new MotorEx(hardwareMap, "outtake", MotorEx.GoBILDA.RPM_312);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getRealVelocity() {
        return motor.getVelocity() * FLYWHEEL_VELOCITY_SCALING_FACTOR;
    }

    public void periodic() {
        double output = controller.calculate(getRealVelocity(), targetVelocity);

        motor.set(output);
    }

    public boolean isTolerance() {
        return controller.atSetPoint();
    }

}
