package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetIMUMode0 extends InstantCommand {

    public VisionSetIMUMode0() {}
  
    @Override
    public void initialize() {
        LimelightVision.getInstance().setIMUMode(0);
    }
}
