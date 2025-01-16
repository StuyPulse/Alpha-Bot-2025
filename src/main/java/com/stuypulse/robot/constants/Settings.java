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
    public interface Shooter {
        double LEFT_SHOOT_SPEED_L1 = 1.0;
        double RIGHT_SHOOT_SPEED_L1 = -1.0;

        double LEFT_SHOOT_SPEED_L2_L3 = 1.0;
        double RIGHT_SHOOT_SPEED_L2_L3 = -1.0;

        double LEFT_SHOOT_SPEED_L4 = 1.0;
        double RIGHT_SHOOT_SPEED_L4 = -1.0;
        
        double LEFT_kP = 0.0;
        double LEFT_kI = 0.0;
        double LEFT_kD = 0.0;

        double RIGHT_kP = 0.0;
        double RIGHT_kI = 0.0;
        double RIGHT_kD = 0.0;
        
        double LEFT_TARGET_RPM = 0.0;
        double RIGHT_TARGET_RPM = 0.0;

        double TARGET_EPSILON = 0.0;
    }
}
