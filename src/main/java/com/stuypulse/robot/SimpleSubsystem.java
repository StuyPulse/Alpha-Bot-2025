package com.stuypulse.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimpleSubsystem extends SubsystemBase {
    private ExamplePIDController pidController;
    private double currentValue;
    private double targetValue;
    private double velocity;

    // ROOKIES: Tune kP, kI, and kD on Glass!
    double kP = 30.0; // 30
    double kI = 0.0;
    double kD = 4.0; // 4

    public static final SimpleSubsystem instance;

    public SimpleSubsystem() {
        currentValue = 0.0;
        targetValue = 100.0;
        velocity = 0.0;

        pidController = new ExamplePIDController(kP, kI, kD);
    }

    static {
        instance = new SimpleSubsystem();
    }

    public static SimpleSubsystem getInstance() {
        return instance;
    }

    public double getCurrentValue() {
        return currentValue;
    }

    public double getTargetValue() {
        return targetValue;
    }

    public void incrementTargetValue(double increment) {
        increment = MathUtil.clamp(increment, -200.0, 200.0);
        targetValue += increment;
    }

    @Override
    public void periodic() {
        double pidOutput = pidController.getOutput(currentValue, targetValue);

        velocity *= 0.9;
        velocity += pidOutput * 0.02;
        currentValue += velocity * 0.02;

        SmartDashboard.putNumber("Rookie Ed: Control Theory/Target Value", targetValue);
        SmartDashboard.putNumber("Rookie Ed: Control Theory/Current Value", currentValue);
        SmartDashboard.putNumber("Rookie Ed: Control Theory/Rate of Change of Value", velocity);

        SmartDashboard.putNumber("Rookie Ed: Control Theory/kP", kP);
        SmartDashboard.putNumber("Rookie Ed: Control Theory/kI", kI);
        SmartDashboard.putNumber("Rookie Ed: Control Theory/kD", kD);
    }
}