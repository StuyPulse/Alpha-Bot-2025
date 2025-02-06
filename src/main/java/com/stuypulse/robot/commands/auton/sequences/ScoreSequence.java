package com.stuypulse.robot.commands.auton.sequences;

import com.stuypulse.robot.commands.elevator.ElevatorToFeed;
import com.stuypulse.robot.commands.elevator.ElevatorToLvl4;
import com.stuypulse.robot.commands.shooter.ShooterShoot;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ScoreSequence extends SequentialCommandGroup {
    
    public ScoreSequence() {
        
        addCommands(

            new ElevatorToLvl4(),
            new ShooterShoot(),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasCoral()),
            new ShooterStop()

        );

    }

}
