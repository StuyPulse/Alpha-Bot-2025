package com.stuypulse.robot;

public class ExampleFFController {
    private double kS, kV, kA;
    private double lastVel;

    public ExampleFFController(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;

        lastVel = 0.0;
    }

    public double getOutput(double velocity_setpoint) {
        double dt = 0.02;
        double output = 
            kS * Math.signum(velocity_setpoint) + 
            velocity_setpoint * kV + 
            kA * ((velocity_setpoint - lastVel) / dt);

        lastVel = velocity_setpoint;
        return output;
    }
}
