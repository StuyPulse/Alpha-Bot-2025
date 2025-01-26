package com.stuypulse.robot.commands.auton.JKLA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.sequences.ScoreSequence;
import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.shooter.ShooterAcquire;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeHalfPieceJKL extends SequentialCommandGroup {
    
    public ThreeHalfPieceJKL(PathPlannerPath... paths) {

        addCommands(

            SwerveDrive.getInstance().followPathCommand(paths[0]),
            new ScoreSequence(),
            new ElevatorToFeed(),

            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new ShooterAcquire(),
            
            SwerveDrive.getInstance().followPathCommand(paths[2]),
            new ScoreSequence(),
            new ElevatorToFeed(),

            SwerveDrive.getInstance().followPathCommand(paths[3]),
            new ShooterAcquire(),
            
            SwerveDrive.getInstance().followPathCommand(paths[4]),
            new ScoreSequence(),
            new ElevatorToFeed(),

            SwerveDrive.getInstance().followPathCommand(paths[5]),
            new ShooterAcquire()
            

        );

    }

}
