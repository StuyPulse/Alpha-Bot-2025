package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;

/**
 * A feedforward term to account for gravity for elevators
 */

public class ElevatorFeedForward extends Controller {

    private final Number kG;

    public ElevatorFeedForward(Number kG) {
        this.kG = kG;
    }

    @Override
    protected double calculate(double setpoint, double measurement) {
        return kG.doubleValue();
    }
}
