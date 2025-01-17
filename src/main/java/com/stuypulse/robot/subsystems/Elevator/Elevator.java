package com.stuypulse.robot.subsystems.Elevator;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings.RobotType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {

    private static final Elevator instance;

    static {
        if (Robot.ROBOT == RobotType.AUNT_MARY) {
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

    public abstract double getHeight();

    public abstract void stopElevator();

    public abstract boolean atBottom();
    
    public void periodic() {
        visualizer.update(getHeight());
    }

}
    