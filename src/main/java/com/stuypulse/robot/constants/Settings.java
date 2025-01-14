/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Algae {
        int ROLLER_ID = 0; // update ports later -- def won't be 0
        int PIVOT_ID = 0; // update ports later
        double TARGET_ANGLE = 0.0; // CHANGE
        double LOWERED_ANGLE = 0.0; // CHANGE
        double RAISED_ANGLE = 0.0; // CHANGE

        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface FF{
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
    }
}
