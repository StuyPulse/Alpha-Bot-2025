package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.subsystems.algae.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlgaeRollerIntake extends InstantCommand {
    
    private final Algae algae; 

    public AlgaeRollerIntake() {
        algae = Algae.getInstance();
    }

    @Override
    public void initialize() {
        algae.acquire();
    }

}