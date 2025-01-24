package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.subsystems.algae.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlgaeStopRoller extends InstantCommand {
    
    private final Algae algae; 

    public AlgaeStopRoller() {
        algae = Algae.getInstance();
        
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.stopRollers();
    }

}