package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;   

public class ElevatorToHeight extends InstantCommand {
    private final Elevator elevator;
    private final Number targetHeight;

    public ElevatorToHeight(double targetHeight){
        elevator = Elevator.getInstance();
        this.targetHeight = targetHeight;
        
        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        elevator.setTargetHeight(targetHeight.doubleValue());
    }
}
