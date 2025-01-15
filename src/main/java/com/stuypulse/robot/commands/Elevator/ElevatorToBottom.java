package com.stuypulse.robot.commands.Elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToBottom extends ElevatorToHeight {

    public ElevatorToBottom() {
        super(Elevator.MIN_HEIGHT);
    }
}
