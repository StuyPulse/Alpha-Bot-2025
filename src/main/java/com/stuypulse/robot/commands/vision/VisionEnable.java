package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionEnable extends InstantCommand {

    public VisionEnable() {}
  
    @Override
    public void initialize() {
        LimelightVision.getInstance().enable();
    }
}
