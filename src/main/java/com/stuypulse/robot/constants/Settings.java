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

    public interface Swerve {
        double MODULE_VELOCITY_DEADBAND = 0.05;

        public interface Constraints {
            double MAX_MODULE_SPEED = 4.9;

            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity (m per s)", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration (m per s^2)", 4.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity (rad per s)", Units.degreesToRadians(540));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());
        }

        public interface Alignment {
            PIDConstants XY = new PIDConstants(2.5, 0, 0.02);
            PIDConstants THETA = new PIDConstants(4, 0, 0.1);

            SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance (m)", 0.05);
            SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance (m)", 0.05);
            SmartNumber THETA_TOLERANCE = new SmartNumber("Alignment/Theta Tolerance (rad)", 0.1);

            double XY_DEBOUNCE = 0.05;
            double THETA_DEBOUNCE = 0.05;
        }

        public interface Turn {
            double kP = 7.0;
            double kI = 0.0;
            double kD = 0.05;
        }

        public interface Drive {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.26722;
            double kV = 2.2119;
            double kA = 0.36249;
        }
    }

    public interface Vision {        
        Vector<N3> STDDEVS = VecBuilder.fill(0.3, 0.3, Math.toRadians(30));
    }

    public interface Elevator {
        SmartNumber MAX_VELOCITY_METERS_PER_SECOND = new SmartNumber("Elevator/Max Velocity (m per s)", 1.0);
        SmartNumber MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = new SmartNumber("Elevator/Max Accel (m per s^2)", 2.0);

        double L1_HEIGHT_METERS = 0;
        double L2_HEIGHT_METERS = 0.25;
        double L3_HEIGHT_METERS = 0.5;
        double L4_HEIGHT_METERS = 0.75;

        double FEED_HEIGHT_METERS = 0.4;

        SmartNumber HEIGHT_TOLERANCE_METERS = new SmartNumber("Elevator/Height Tolerance (m)", 0.02);
    
        public interface PID {
            SmartNumber kP = new SmartNumber("Elevator/Controller/kP",10);
            SmartNumber kI = new SmartNumber("Elevator/Controller/kI",0.0);
            SmartNumber kD = new SmartNumber("Elevator/Controller/kD",0.2);
        }

        public interface FF {
            SmartNumber kS = new SmartNumber("Elevator/Controller/kS",0.20506);
            SmartNumber kV = new SmartNumber("Elevator/Controller/kV",3.7672);
            SmartNumber kA = new SmartNumber("Elevator/Controller/kA", 0.27);
            SmartNumber kG = new SmartNumber("Elevator/Controller/kG", 1.37);
        }
        
        public interface Simulation {
            double SCALE_FACTOR = 0.5 + 2.5/77;
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

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.Constraints.MAX_VELOCITY.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.Constraints.MAX_ACCELERATION.get());
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad per s)", Swerve.Constraints.MAX_ANGULAR_VELOCITY.get());
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad per s^2)", Swerve.Constraints.MAX_ANGULAR_ACCELERATION.get());
        }
    }
}
