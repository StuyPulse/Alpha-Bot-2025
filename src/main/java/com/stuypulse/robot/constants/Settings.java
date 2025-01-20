/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

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

    double DT = 0.020;

    public interface Robot {
        double kG = 100.0;
    }

    public interface Elevator {
        double MIN_HEIGHT = 0.0;
        double MAX_HEIGHT = 12.0;
        double MAX_ACCELERATION = 2.0;
        double MAX_VELOCITY = 3.0;
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
            SmartNumber kP = new SmartNumber("kP",1.5);
            SmartNumber kI = new SmartNumber("kI",0.0);
            SmartNumber kD = new SmartNumber("kD",0.2);
        }

        public interface FF {
            SmartNumber kS = new SmartNumber("kS",0.20506);
            SmartNumber kV = new SmartNumber("kV",3.7672);
            SmartNumber kA = new SmartNumber("kA", 0.27);
            SmartNumber kG = new SmartNumber("kG", 0.37);
        }
    }
}


