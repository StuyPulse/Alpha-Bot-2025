package com.stuypulse.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ExampleAuto extends SequentialCommandGroup {
    
    public ExampleAuto(PathPlannerPath... paths) {

        addCommands(
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new WaitCommand(1.0),
            SwerveDrive.getInstance().followPathCommand(paths[1])
        );

    }

}
