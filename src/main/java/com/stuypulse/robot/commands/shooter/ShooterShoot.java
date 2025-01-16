package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterShoot extends InstantCommand {
    private final Shooter shooter;

    public ShooterShoot() {
        shooter = Shooter.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterState(ShooterState.SHOOTING);
    }
}