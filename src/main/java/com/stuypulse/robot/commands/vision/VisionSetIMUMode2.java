package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetIMUMode2 extends InstantCommand {

    public VisionSetIMUMode2() {}
  
    @Override
    public void initialize() {
        LimelightVision.getInstance().setIMUMode(2);
    }
}
