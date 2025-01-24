/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

public interface Settings {

    double DT = 0.020;

    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30); // TODO: find width
    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(30); // TODO: find length

    public interface Elevator {
        double MIN_HEIGHT_METERS = 0.0; // FROM THE FLOOR TO TOP OF CARRIAGE
        double MAX_HEIGHT_METERS = 12.0; // FROM THE FLOOR TO TOP OF CARRIAGE
        double MAX_VELOCITY_METERS_PER_SECOND = 3.0;
        double MAX_ACCELERATION_METERS_PER_SECOND = 2.0;
        
        double MASS = 25.0;
        double GEARING = 1.0/9.0;
        double DRUM_RADIUS = Units.inchesToMeters(1.0);

        double L1_HEIGHT_METERS = 0;
        double L2_HEIGHT_METERS = 0.25;
        double L3_HEIGHT_METERS = 0.5;
        double L4_HEIGHT_METERS = 0.75;

        public interface Encoders {
            double POSITION_CONVERSION_FACTOR = 1.0;
            double VELOCITY_CONVERSION_FACTOR = 1.0;
        }
    
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
        
        public interface Simulation {
            double SCALE_FACTOR = 0.5;
        }
    }

    public interface Algae {
        // 0 degrees is as far into the robot as the arm can go
        SmartNumber GROUND_PICKUP_ANGLE_DEGREES = new SmartNumber("Algae Mech/Ground Pickup Angle (deg)", 0);
        SmartNumber L2_ANGLE_DEGREES = new SmartNumber("Algae Mech/L2 Angle (deg)", 0);
        SmartNumber L3_ANGLE_DEGREES = new SmartNumber("Algae Mech/L3 Angle (deg)", 0);
        SmartNumber PROCESSOR_ANGLE_DEGREES = new SmartNumber("Algae Mech/Processor Angle (deg)", 0);
        SmartNumber ACQUIRE_SPEED = new SmartNumber("Algae Mech/Acquire Speed", 0.5);
        SmartNumber DEACQUIRE_SPEED = new SmartNumber("Algae Mech/Deacquire Speed", 0.5);

        double ENCODER_OFFSET_DEGREES = 0;

        double MAX_ANGULAR_VELOCITY_RAD_PER_SECOND = 3.0;
        double MAX_ANGULAR_ACCEL_RAD_PER_SECOND_PER_SECOND = 3.0;
    
        public interface PID {
            SmartNumber kP = new SmartNumber("Algae Mech/PID/kP", 0.0);
            SmartNumber kI = new SmartNumber("Algae Mech/PID/kI", 0.0);
            SmartNumber kD = new SmartNumber("Algae Mech/PID/kD", 0.0);
        }

        public interface FF{
            SmartNumber kS = new SmartNumber("Algae Mech/FF/kS", 0.0);
            SmartNumber kV = new SmartNumber("Algae Mech/FF/kV", 0.0);
            SmartNumber kA = new SmartNumber("Algae Mech/FF/kA", 0.0);
            SmartNumber kG = new SmartNumber("Algae Mech/FF/kG", 0.0);
        }
    }

    public interface Shooter {
        SmartNumber ACQUIRE_SPEED = new SmartNumber("Shooter/Acquire Speed", 0.2);
        SmartNumber SHOOT_SPEED = new SmartNumber("Shooter/Shoot Speed", 0.5);
    }
}
