package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelStop extends Command{
    
    private final Funnel funnel;

    public FunnelStop() {
        this.funnel = Funnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void initialize() {
        funnel.stop();
    }
}
