package org.firstinspires.ftc.teamcode.util;

public class PIDF {
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double integral;
    private double previousError;

    private double windupRange;
    private boolean signFlipReset = true;

    private long lastTime;

    public PIDF(double kP, double kI, double kD, double kF, double windupRange) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        this.windupRange = windupRange;

        this.integral = 0;
        this.previousError = 0;
        this.lastTime = System.currentTimeMillis();
    }

    public double calculate(double target, double current) {
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        double error = target - current;

        if (dt > 0) {
            if (Math.signum(error) != Math.signum(previousError) && signFlipReset) {
                integral = 0;
            }

            if (windupRange != 0 && Math.abs(error) > windupRange) {
                integral = 0;
            } else {
                integral += error * dt;
            }
        }

        double derivative = 0;
        if (dt > 0)
            derivative = (error - previousError) / dt;

        previousError = error;

        return (kP * error)
                + (kI * integral)
                + (kD * derivative)
                + (kF * target);
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        lastTime = System.currentTimeMillis();
    }
}