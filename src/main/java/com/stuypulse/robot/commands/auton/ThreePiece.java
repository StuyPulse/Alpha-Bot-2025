package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.sequences.AcquireSequence;
import com.stuypulse.robot.commands.auton.sequences.ScoreSequence;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreePiece extends SequentialCommandGroup {
    
    public ThreePiece(PathPlannerPath... paths) {

        addCommands(

            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new ScoreSequence(),

            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new AcquireSequence(),
            
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            new ScoreSequence(),

            SwerveDrive.getInstance().followPathCommand(paths[3]),
            new AcquireSequence(),
            
            SwerveDrive.getInstance().followPathCommand(paths[4]),
            new ScoreSequence()
            

        );

    }

}
