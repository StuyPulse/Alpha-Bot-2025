package com.stuypulse.robot;

public class ExamplePIDController {
    private double kP, kI, kD;
    private double integral, lastError;

    public ExamplePIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        integral = 0;
        lastError = 0;
    }

    public double getOutput(double measurement, double setpoint) {
        double error = setpoint - measurement;
        double dt = 0.02;

        integral += error * dt * kI;
        double output = (kP * error) + (integral) + (kD * (lastError - error) / dt);

        lastError = error;
        return output;
    }
}
