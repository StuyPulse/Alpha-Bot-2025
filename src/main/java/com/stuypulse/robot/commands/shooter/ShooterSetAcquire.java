package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSetAcquire extends InstantCommand {
    private final Shooter shooter;
    // private final Gamepad driver;

    public ShooterSetAcquire() {
        shooter = Shooter.getInstance();
        //this.driver = driver;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.acquire();
    }

    // @Override
    // public boolean isFinished() {
    //     return driver.getRawRightMenuButton();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //    shooter.stop();
    // }
}