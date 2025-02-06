package com.stuypulse.robot.commands.auton.misc;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.sequences.ScoreSequence;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OnePieceH extends SequentialCommandGroup {
    
    public OnePieceH(PathPlannerPath... paths) {

        addCommands(

            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new ScoreSequence(),
            new ElevatorToFeed()

        );

    }

}
