package com.stuypulse.robot.subsystems.Elevator;

import java.util.Optional;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorSimu extends Elevator {
    private final ElevatorSim sim;
    private final SmartNumber targetHeight;
    private final double minHeight, maxHeight;
    private final SmartNumber maxAccel, maxVel;
    private Optional<Double> voltageOverride;
    private SimpleMotorFeedforward FF;
    private PIDController PID;
    
    ElevatorSimu() {
        targetHeight = new SmartNumber("Elevator/Target Height", 0);
        minHeight = Settings.Elevator.MIN_HEIGHT;
        maxHeight = Settings.Elevator.MAX_HEIGHT;
        maxAccel = new SmartNumber("Elevator/Max Acceleration",Settings.Elevator.MAX_ACCELERATION);
        maxVel = new SmartNumber("Elevator/Max Velocity", Settings.Elevator.MAX_VELOCITY);
        sim = new ElevatorSim(
            DCMotor.getNEO(2),
            Settings.Elevator.GEARING,
            Settings.Elevator.MASS,
            Settings.Elevator.DRUM_RADIUS,
            Settings.Elevator.MIN_HEIGHT,
            Settings.Elevator.MAX_HEIGHT,
            true,
            0.0);
        FF = new SimpleMotorFeedforward(Settings.Elevator.FF.kS, Settings.Elevator.FF.kV, Settings.Elevator.FF.kA);
        PID = new PIDController(Settings.Elevator.PID.kP, Settings.Elevator.PID.kI, Settings.Elevator.PID.kD);
        voltageOverride = Optional.empty();
    }

    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(SLMath.clamp(height, minHeight, maxHeight));
        voltageOverride = Optional.empty();
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.doubleValue();
    }

    @Override
    public boolean atBottom() {
        return sim.hasHitLowerLimit();
    }
    
    public boolean elevatorTop() {
        return sim.hasHitUpperLimit();
    }

    @Override
    public double getHeight() {
        return sim.getPositionMeters();
    }

    @Override
    public void stopElevator() {
        sim.setInputVoltage(0.0);
    }
    public void elevatorIdleMode(IdleMode mode) {}
    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
    }
    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVel.set(maxVelocity);
        this.maxAccel.set(maxAcceleration);
    }
    public double calculateVoltage() {
        final double FFOutput = FF.calculate(Units.metersToInches(sim.getVelocityMetersPerSecond()));
        final double PIDOutput = PID.calculate(getHeight(), targetHeight.doubleValue());
        // Combine outputs and set motor voltage
        return FFOutput + PIDOutput;
    }
    
    public void periodic() {
        super.periodic();
        double voltage = voltageOverride.orElse(calculateVoltage());
        
        if (atBottom() && voltage < 0 || elevatorTop() && voltage > 0) {
            stopElevator();
        } else {
            sim.setInputVoltage(voltage);
        }
        
        SmartDashboard.putNumber("Elevator/Target Height", getTargetHeight());
        SmartDashboard.putNumber("Elevator/Current", sim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Elevator/Height", getHeight());
    }

    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
        sim.update(Settings.DT);
    }
}