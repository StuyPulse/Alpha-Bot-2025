package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.subsystems.algae.Algae;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeAcquireUnder extends Command{
    
    private final Algae algae;

    public AlgaeAcquireUnder() {
        this.algae = Algae.getInstance();
        addRequirements(algae);
    }

    @Override
    public void initialize() {
        algae.acquireUnder();
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
