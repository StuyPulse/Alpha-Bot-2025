package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Constants.Elevator;

public class ElevatorToTop extends ElevatorToHeight {

    public ElevatorToTop() {
        super(Elevator.MAX_HEIGHT_METERS);
    }
}
