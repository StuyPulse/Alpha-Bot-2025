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
        double MAX_HEIGHT = Units.inchesToMeters(86);
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

        double STAGE_TWO_SCALE_FACTOR = 0.5 + 3.5/86;
    
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
