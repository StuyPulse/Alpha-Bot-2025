package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelDefaultCommand extends Command{
    
    private final Funnel funnel;

    public FunnelDefaultCommand() {
        this.funnel = Funnel.getInstance();
        addRequirements(funnel);
    }

    @Override
    public void execute() {
        if (Shooter.getInstance().hasCoral()) {
            funnel.stop();
        }
        else if (funnel.shouldReverse()) {
            funnel.reverse();
        }
        else {
            funnel.acquire();
        }
    }
}
