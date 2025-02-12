package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetIMUMode1 extends InstantCommand {

    public VisionSetIMUMode1() {}
  
    @Override
    public void initialize() {
        LimelightVision.getInstance().setIMUMode(1);
    }
}
