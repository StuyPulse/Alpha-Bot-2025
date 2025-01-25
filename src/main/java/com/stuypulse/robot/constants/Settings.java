/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public interface Settings {

    double DT = 0.020;
  
    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(29);
    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(29);

    public interface Swerve {
        // between wheel centers
        double WIDTH = Units.inchesToMeters(20.75);
        double LENGTH = Units.inchesToMeters(20.75);

        double MAX_MODULE_SPEED = 4.9;

        double MODULE_VELOCITY_DEADBAND = 0.05;

        public interface Assist {
            // angle PID
            SmartNumber kP = new SmartNumber("SwerveAssist/kP", 6.0);
            SmartNumber kI = new SmartNumber("SwerveAssist/kI", 0.0);
            SmartNumber kD = new SmartNumber("SwerveAssist/kD", 0.0);

            double ANGLE_DERIV_RC = 0.05;
            double REDUCED_FF_DIST = 0.75;
        }

        // TODO: Tune these values
        public interface Motion {
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration", 4.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity", Units.degreesToRadians(540));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());

            PIDConstants XY = new PIDConstants(2.5, 0, 0.02);
            PIDConstants THETA = new PIDConstants(4, 0, 0.1);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 5.36;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            }
        }

        public interface Turn {
            double kP = 7.0;
            double kI = 0.0;
            double kD = 0.05;

            double kS = 0.25582;
            double kV = 0.00205;
            double kA = 0.00020123;
        }

        public interface Drive {
            double kP = 0.31399;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.15618;
            double kV = 0.00012288;
            double kA = 0.0000259;
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(0.0);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
        }
    }

    public interface Vision {        
        Vector<N3> STDDEVS = VecBuilder.fill(0.3, 0.3, Math.toRadians(30));
    }

    public interface Elevator {
        double MIN_HEIGHT_METERS = Units.inchesToMeters(9.09375); // FROM THE BOTTOM OF FIXED STAGE TO TOP OF CARRIAGE
        double MAX_HEIGHT_METERS = Units.inchesToMeters(77); // FROM THE BOTTOM OF FIXED STAGE TO TOP ELEVATOR
        double MAX_VELOCITY_METERS_PER_SECOND = 3.0;
        double MAX_ACCELERATION_METERS_PER_SECOND = 2.0;
        double MASS_KG = 25.0;

        double L1_HEIGHT_METERS = 0;
        double L2_HEIGHT_METERS = 0.25;
        double L3_HEIGHT_METERS = 0.5;
        double L4_HEIGHT_METERS = 0.75; // All in meters

        double FEED_HEIGHT_METERS = 0.4;

        public interface Encoders {
            double GEARING = 4.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (6 + 9.0 / 24) * GEARING; // Number of rotations that the motor has to spin, NOT the gear

            double DRUM_CIRCUMFERENCE_METERS = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP * GEARING;
            double DRUM_RADIUS_METERS = DRUM_CIRCUMFERENCE_METERS / 2 / Math.PI;

            double POSITION_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP / 60;
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
            double SCALE_FACTOR = 0.5 + 2.5/87;
        }
    }

    public interface Shooter {
        SmartNumber ACQUIRE_SPEED = new SmartNumber("Shooter/Acquire Speed", 0.2);
        SmartNumber SHOOT_SPEED = new SmartNumber("Shooter/Shoot Speed", 0.5);
    }

    public interface Funnel {
        SmartNumber ACQUIRE_SPEED = new SmartNumber("Funnel/Acquire Speed", 0.4);
        SmartNumber DEACQUIRE_SPEED = new SmartNumber("Funnel/Deacquire Speed", 0.4); 
    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.Motion.MAX_VELOCITY.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.Motion.MAX_ACCELERATION.get());
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad/s)", Swerve.Motion.MAX_ANGULAR_VELOCITY.get());
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad/s^2)", Swerve.Motion.MAX_ANGULAR_ACCELERATION.get());
        }
    }
}
