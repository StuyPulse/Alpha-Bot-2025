/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }
  
    public interface Swerve {
        public interface FrontRight {
            int DRIVE = 10;
            int TURN = 11;
            int ENCODER = 1;
        }

        public interface FrontLeft {
            int DRIVE = 12;
            int TURN = 13;
            int ENCODER = 2;
        }

        public interface BackLeft {
            int DRIVE = 14;
            int TURN = 15;
            int ENCODER = 3;
        }

        public interface BackRight {
            int DRIVE = 16;
            int TURN = 17;
            int ENCODER = 4;
        }

    public interface Shooter {
        int MOTOR = 0;
        int IR_SENSOR = 1;
    }

    public interface Elevator {
        int LEFT = 0;
        int RIGHT = 1;
        int SWITCH = 2;
    }
  
    public interface Algae {
        int ROLLER = 0;
        int PIVOT = 1;
        int ENCODER = 2;
    }

    public interface Funnel {
        int MOTOR = 0;
    }
}

