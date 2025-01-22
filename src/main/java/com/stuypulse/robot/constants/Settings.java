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
        double RAISED_ANGLE = 0.0;          // CHANGE
        double PROCESSOR_ANGLE = 0.0;       // CHANGE 
        double REEF_KNOCKOFF_ANGLE = 0.0;   // CHANGE
        double GROUND_PICKUP_ANGLE = 0.0;   // CHANGE
        double L2_ANGLE = 0.0;              // CHANGE
        double L3_ANGLE = 0.0;              // CHANGE
        double STOW_ANGLE = 0.0;            // CHANGE 
        double ACQUIRE_SPEED = 0.0;         // CHANGE
        double DEACQUIRE_SPEED = 0.0;       // CHANGE
    
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
            double MAX_VELOCITY = 0.0;
            double MAX_ACCELERATION = 0.0;
        }
        

        public interface FF{
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
    }
}

