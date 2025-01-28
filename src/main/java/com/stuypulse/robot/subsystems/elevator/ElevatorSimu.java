package com.stuypulse.robot.subsystems.elevator;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSimu extends Elevator {

    private final ElevatorSim sim;
    private final double minHeight, maxHeight;

    private final SmartNumber targetHeight;

    private final Controller controller;
    
    ElevatorSimu() {

        sim = new ElevatorSim(
            DCMotor.getNEO(2),
            Settings.Elevator.Encoders.GEARING,
            Settings.Elevator.MASS_KG,
            Settings.Elevator.Simulation.DRUM_RADIUS_METERS,
            Settings.Elevator.MIN_HEIGHT_METERS,
            Settings.Elevator.MAX_HEIGHT_METERS,
            true,
            Settings.Elevator.MIN_HEIGHT_METERS
        );
        
        minHeight = Settings.Elevator.MIN_HEIGHT_METERS;
        maxHeight = Settings.Elevator.MAX_HEIGHT_METERS;

        targetHeight = new SmartNumber("Elevator/Target Height (m)", Settings.Elevator.MIN_HEIGHT_METERS);
        
        MotionProfile motionProfile = new MotionProfile(Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND, Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND);
        
        controller = new MotorFeedforward(Settings.Elevator.FF.kS, Settings.Elevator.FF.kV, Settings.Elevator.FF.kA).position()
            .add(new ElevatorFeedforward(Settings.Elevator.FF.kG))
            .add(new PIDController(Settings.Elevator.PID.kP, Settings.Elevator.PID.kI, Settings.Elevator.PID.kD))
            .setSetpointFilter(motionProfile);
    }

    public ElevatorSim getSim() {
        return sim;
    }
    
    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(SLMath.clamp(height, minHeight, maxHeight));
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.get();
    }

    @Override
    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }
    
    @Override
    public void periodic() {
        super.periodic();
        
        controller.update(getTargetHeight(), getCurrentHeight());
        sim.setInputVoltage(controller.getOutput());
        sim.update(Settings.DT);
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));
        
        ElevatorVisualizer.getVisualizerInstance().update();

        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
    }
}
