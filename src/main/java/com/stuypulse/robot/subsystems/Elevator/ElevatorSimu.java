package com.stuypulse.robot.subsystems.elevator;

import java.util.Optional;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSimu extends Elevator {

    private final ElevatorSim sim;
    private final double minHeight, maxHeight;

    private final SmartNumber targetHeight;
    private final SmartNumber maxAccel, maxVel;

    private ElevatorFeedforward FF;
    private PIDController PID;

    private Optional<Double> voltageOverride;
    
    ElevatorSimu() {

        sim = new ElevatorSim(
            DCMotor.getNEO(2),
            Settings.Elevator.Encoders.GEARING,
            Settings.Elevator.MASS_KG,
            Settings.Elevator.Encoders.DRUM_RADIUS_METERS,
            Settings.Elevator.MIN_HEIGHT_METERS,
            Settings.Elevator.MAX_HEIGHT_METERS,
            true,
            0.0
        );
        
        minHeight = Settings.Elevator.MIN_HEIGHT_METERS;
        maxHeight = Settings.Elevator.MAX_HEIGHT_METERS;

        targetHeight = new SmartNumber("Elevator/Target Height", 0);
        maxAccel = new SmartNumber("Elevator/Max Acceleration", Settings.Elevator.MAX_ACCELERATION_METERS_PER_SECOND);
        maxVel = new SmartNumber("Elevator/Max Velocity", Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND);
        
        FF = new ElevatorFeedforward(
            Settings.Elevator.FF.kS.getAsDouble(), 
            Settings.Elevator.FF.kG.getAsDouble(), 
            Settings.Elevator.FF.kV.getAsDouble(), 
            Settings.Elevator.FF.kA.getAsDouble()
        );

        PID = new PIDController(
            Settings.Elevator.PID.kP.getAsDouble(), 
            Settings.Elevator.PID.kI.getAsDouble(), 
            Settings.Elevator.PID.kD.getAsDouble()
        );

        voltageOverride = Optional.empty();

    }

    public ElevatorSim getSim() {
        return sim;
    }
    
    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(SLMath.clamp(height, minHeight, maxHeight));
        voltageOverride = Optional.empty();
    }

    @Override
    public double getTargetHeight() {
        setTargetHeight(targetHeight.doubleValue());
        return Units.inchesToMeters(targetHeight.doubleValue());
    }

    public boolean atBottom() {
        return sim.hasHitLowerLimit();
    }
    
    public boolean elevatorTop() {
        return sim.hasHitUpperLimit();
    }

    @Override
    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }

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
        final double FFOutput = FF.calculate(PID.getSetpoint());
        final double PIDOutput = PID.calculate(getCurrentHeight(), targetHeight.doubleValue());

        return FFOutput + PIDOutput;
    }
    
    public void periodic() {

        super.periodic();
        
        double voltage = calculateVoltage();

        if (atBottom() && voltage < 0 || elevatorTop() && voltage > 0) {
            stopElevator();
        } else {
            sim.setInputVoltage(voltage);
        }
        
        ElevatorVisualizer.getVisualizerInstance().update();

        SmartDashboard.putNumber("Elevator/Target Height", getTargetHeight());
        SmartDashboard.putNumber("Elevator/Current", sim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Elevator/Height", getCurrentHeight());
    }

    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
        sim.update(Settings.DT);
    }
}
