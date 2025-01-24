package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.subsystems.algae.Algae;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeAcquireOver extends Command{
    
    private final Algae algae;

    public AlgaeAcquireOver() {
        this.algae = Algae.getInstance();
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.acquireOver();
    }

    @Override
    public boolean isFinished() {
        return algae.hasAlgae();
    }

    @Override
    public void end(boolean interrupted) {
        algae.stopRollers();
    }
}
