package com.stuypulse.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ExampleShooterCommand extends InstantCommand {

    private ExampleShooterSim sim;
    private double targetRPM;

    public ExampleShooterCommand(double targetRPM) {
        sim = ExampleShooterSim.getInstance();
        this.targetRPM = targetRPM;
        addRequirements(sim);
    }

    @Override
    public void initialize() {
        sim.setTargetRPM(targetRPM);
    }
}
