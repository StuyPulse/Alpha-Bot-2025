package com.stuypulse.robot.subsystems.Elevator;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {

    private static final Elevator instance;

    static {
        if (Robot.isReal()) {
            instance = new ElevatorImpl();
        } else {
            instance = new ElevatorSimu();
        }
    }

    private final ElevatorVisualizer visualizer;

    public static Elevator getInstance() {
        return instance;
    }    

    public Elevator() {
        visualizer = new ElevatorVisualizer();
    }

    public abstract void setTargetHeight(double height);
    public abstract double getTargetHeight();
    public abstract double getCurrentHeight();
    
    public void periodic() {
        visualizer.update();
    }

}
    