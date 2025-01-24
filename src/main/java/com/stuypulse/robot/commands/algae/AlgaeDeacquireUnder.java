package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.subsystems.algae.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AlgaeDeacquireUnder extends InstantCommand {
    
    private final Algae algae; 

    public AlgaeDeacquireUnder() {
        algae = Algae.getInstance();
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.deacquireUnder();
    }

}