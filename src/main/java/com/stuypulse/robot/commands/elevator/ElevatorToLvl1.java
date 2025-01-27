package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToLvl1 extends ElevatorToHeight{
    public ElevatorToLvl1(){
        super(Elevator.L1_HEIGHT_METERS);
    }
}
