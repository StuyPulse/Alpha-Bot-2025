/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public interface Settings {

    double DT = 0.020;

    public interface Swerve {
        String DRIVE_CANBUS = "Swerve Drive Drive";
        double MODULE_VELOCITY_DEADBAND = 0.05;

        public interface Constraints {
            double MAX_MODULE_SPEED = 4.9;

            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity (m per s)", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration (m per s^2)", 5.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity (rad per s)", Units.degreesToRadians(360));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());
        }

        public interface Alignment {
            PIDConstants XY = new PIDConstants(2.0, 0, 0.02);
            PIDConstants THETA = new PIDConstants(3, 0, 0.1);

            SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance (m)", 0.05);
            SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance (m)", 0.05);
            SmartNumber THETA_TOLERANCE = new SmartNumber("Alignment/Theta Tolerance (rad)", 0.1);

            double ALIGNMENT_DEBOUNCE = 0.05;
        }
    }

    public interface Vision {
        double MIN_DATAPOINTS_BEFORE_MT2 = 500;
        Vector<N3> MIN_STDDEVS = VecBuilder.fill(0.3, 0.3, 5);
    }

    public interface Elevator {
        SmartNumber MAX_VELOCITY_METERS_PER_SECOND = new SmartNumber("Elevator/Max Velocity (m per s)", 2.5);
        SmartNumber MAX_ACCEL_METERS_PER_SECOND_PER_SECOND = new SmartNumber("Elevator/Max Accel (m per s^2)", 5.0);

        double RESET_STALL_CURRENT = 30;

        double RESET_VOLTAGE = 3;
        double MAX_VOLTAGE = 6;

        SmartNumber L2_HEIGHT_METERS = new SmartNumber("Elevator/L2 Height (m)", 1.302004);
        SmartNumber L3_HEIGHT_METERS = new SmartNumber("Elevator/L3 Height (m)", 1.805820);
        SmartNumber L4_HEIGHT_METERS = new SmartNumber("Elevator/L4 Height (m)", 2.627014);

        double FEED_HEIGHT_METERS = Constants.Elevator.MIN_HEIGHT_METERS;

        SmartNumber HEIGHT_TOLERANCE_METERS = new SmartNumber("Elevator/Height Tolerance (m)", 0.03);
        
        public interface Simulation {
            double SCALE_FACTOR = 0.5 + 2.5/77;
        }
    }

    public interface Shooter {
        double HAS_CORAL_DEBOUNCE = 0.0;
        public interface Top {
            SmartNumber ACQUIRE_SPEED = new SmartNumber("Shooter/Top Acquire Speed", 0.22);
            SmartNumber SHOOT_SPEED = new SmartNumber("Shooter/Top Shoot Speed", 0.7);
        }
        public interface Bottom {
            SmartNumber ACQUIRE_SPEED = new SmartNumber("Shooter/Bottom Acquire Speed", 0.22);
            SmartNumber SHOOT_SPEED = new SmartNumber("Shooter/Bottom Shoot Speed", 0.7);
        }
    }

    public interface Funnel {
        SmartNumber ACQUIRE_SPEED = new SmartNumber("Funnel/Acquire Speed", 0.4);
        SmartNumber REVERSE_SPEED = new SmartNumber("Funnel/Reverse Speed", 0.4); 

        double STALL_CURRENT = 25.0;
        double STALL_DETECTION_TIME = 0.25;
        double MIN_REVERSE_TIME = 1.0;
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
