package com.stuypulse.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IncrementSimpleTarget extends InstantCommand {
    private SimpleSubsystem system;
    private double increment;

    public IncrementSimpleTarget(double increment) {
        this.increment = increment;
        system = SimpleSubsystem.getInstance();
        addRequirements(system);
    }

    public void initialize() {
        system.incrementTargetValue(increment);
    }
}
