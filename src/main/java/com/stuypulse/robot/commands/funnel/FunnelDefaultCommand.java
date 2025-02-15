package com.stuypulse.robot.commands.funnel;

import com.stuypulse.robot.subsystems.funnel.Funnel;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelDefaultCommand extends Command{
    
    private final Funnel funnel;
    private final Gamepad driver;

    public FunnelDefaultCommand(Gamepad driver) {
        this.funnel = Funnel.getInstance();
        this.driver = driver;
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

    @Override
    public boolean isFinished() {
        return driver.getRawDPadRight();
    }

    @Override
    public void end(boolean interrupted) {
       funnel.stop();
    }
}
