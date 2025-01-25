package com.stuypulse.robot.commands.auton.sequences;

import com.stuypulse.robot.commands.Elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.Elevator.ElevatorToLvl4;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.shooter.ShooterStop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreSequence extends SequentialCommandGroup {
    
    public ScoreSequence() {
        
        addCommands(

            new ElevatorToLvl4(),
            new ShooterShoot(),
            new WaitCommand(0.25),
            new ShooterStop(),
            new ElevatorToFeed()

        );

    }

}
