package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAcquire extends Command{
    
    private final Shooter shooter;

    public ShooterAcquire() {
        this.shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.acquire();
    }

    @Override
    public boolean isFinished() {
        return shooter.hasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
