package com.stuypulse.robot.commands.auton.sequences;

import com.stuypulse.robot.commands.funnel.FunnelAcquire;
import com.stuypulse.robot.commands.funnel.FunnelStop;
import com.stuypulse.robot.commands.shooter.ShooterAcquire;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AcquireSequence extends SequentialCommandGroup {

    public AcquireSequence() {

        addCommands(

            new FunnelAcquire(),
            new WaitCommand(025),
            new FunnelStop(),
            new ShooterAcquire()

    );

    }

}
