package com.stuypulse.robot.commands.auton.tests;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RSquareTest extends SequentialCommandGroup {

    public RSquareTest(PathPlannerPath... paths) {
        
        addCommands(

            SwerveDrive.getInstance().followPathCommand(paths[0]),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            SwerveDrive.getInstance().followPathCommand(paths[3])

        );

    }
    
}
