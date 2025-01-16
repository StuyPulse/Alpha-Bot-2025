/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    // checks the current RIO's serial number to determine which robot is running
    public enum RobotType {

        AUNT_MARY("0000000"),
        SIM("");

        public final String serialNum;

        RobotType(String serialNum) {
            this.serialNum = serialNum;
        }

        public static RobotType fromString(String serialNum) {
            for (RobotType robot : RobotType.values()) {
                if (robot.serialNum.equals(serialNum.toUpperCase())) {
                    return robot;
                }
            }

            return RobotType.SIM;
        }
    }

    double DT = 1.0/50.0;

    public interface Robot {
        double kG = 100.0;
    }

    public interface Elevator {
        double MIN_HEIGHT = 0.0;
        double MAX_HEIGHT = 12.0;
        double MAX_ACCELERATION = 0.5;
        double MAX_VELOCITY = 1.0;
        double ENCODER_CONVERSION_FACTOR = 0;
        
        double MASS = 25.0;
        double GEARING = 1.0/9.0;
        double DRUM_RADIUS = Units.inchesToMeters(1.0);

        double L1 = 1;
        double L2 = 2;
        double L3 = 3;
        double L4 = 4;
         
        double POSITION_CONVERSION_FACTOR = 1.0;
        double VELOCITY_CONVERSION_FACTOR = 1.0;

        double SCALE_FACTOR = 0.5;
    
        public interface PID {
            double kP = 1.0;
            double kI = 1.0;
            double kD = 1.0;
        }

        public interface FF {
            double kS = 0.1;
            double kV = 0.1;
            double kA = 0.0;
            double kG = 0.1;
        }
    }
}


