package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelDeacquire extends InstantCommand{
    
    private final Funnel funnel;

    public FunnelDeacquire() {
        this.funnel = Funnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.deacquire();
    }
}
