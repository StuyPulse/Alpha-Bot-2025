package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.subsystems.algae.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlgaeSetAngle extends InstantCommand {
    
    private final Algae algae; 
    private final Number targetAngle;

    public AlgaeSetAngle(Number targetAngle) {
        algae = Algae.getInstance();

        this.targetAngle = targetAngle;
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.setTargetAngle(targetAngle.doubleValue());
    }

}