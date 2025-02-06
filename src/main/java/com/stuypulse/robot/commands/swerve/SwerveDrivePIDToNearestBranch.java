package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;

public class SwerveDrivePIDToNearestBranch extends SwerveDrivePIDToPose{
    public SwerveDrivePIDToNearestBranch() {
        super(() -> Field.getClosestBranch().getTargetPose());
    }
}
