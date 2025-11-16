package com.stuypulse.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleShooterSim extends SubsystemBase {
    private ExamplePIDController pidController;
    private ExampleFFController ffController;
    private FlywheelSim shooter;
    private double targetRPM;

    private static final double MAX_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE = -12.0;
    private static final double GEAR_RATIO = 2.0;

    public static final ExampleShooterSim instance;

    public ExampleShooterSim() {
        LinearSystem<N1, N1, N1> system = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1), 
            0.05,
            GEAR_RATIO
        );

        shooter = new FlywheelSim(system, DCMotor.getNEO(1), GEAR_RATIO);
        targetRPM = 5000.0;

        double kS = 0.05;
        double kV = 0.002;
        double kA = 0.01;

        // ROOKIES: Tune these values below and see how your controller performs!

        double kP = 0.01;
        double kI = 0.1;
        double kD = 0.002;

        ffController = new ExampleFFController(kS, kV, kA);
        pidController = new ExamplePIDController(kP, kI, kD);
    }

    static {
        instance = new ExampleShooterSim();
    }

    public static ExampleShooterSim getInstance() {
        return instance;
    }

    public double getShooterRPM() {
        return shooter.getAngularVelocityRPM() / GEAR_RATIO;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public void setTargetRPM(double targetRPM) {
        this.targetRPM = targetRPM;
    }

    @Override
    public void periodic() {
        // double ff_output = ffController.getOutput(getTargetRPM());
        double pid_output = pidController.getOutput(getShooterRPM(), getTargetRPM());
        double total_voltage = pid_output;

        double clamped_voltage = MathUtil.clamp(total_voltage, MIN_VOLTAGE, MAX_VOLTAGE);
        
        shooter.setInputVoltage(clamped_voltage);

        shooter.update(0.02);

        SmartDashboard.putNumber("Rookie Ed: Control Theory/Target RPM ", getTargetRPM());
        SmartDashboard.putNumber("Rookie Ed: Control Theory/Shooter RPM", getShooterRPM());
    }

}
