package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*
 * File containing all the robot constants
 *
 * Such constants include:
 *  - Length and width
 *  - Max and min heights for a elevator
 * 
 * This file is intended to make it easier to double check constants with engineering since everything is in one file
 */
public interface Constants {

    double LENGTH_WITH_BUMPERS_METERS = Units.inchesToMeters(29);
    double WIDTH_WITH_BUMPERS_METERS = Units.inchesToMeters(29);

    public interface Swerve {
        double WIDTH = Units.inchesToMeters(20.75);
        double LENGTH = Units.inchesToMeters(20.75);

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / 5.36;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
            }
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

    public interface Elevator {
        double MIN_HEIGHT_METERS = Units.inchesToMeters(9.09375); // FROM THE BOTTOM OF FIXED STAGE TO TOP OF CARRIAGE
        double MAX_HEIGHT_METERS = Units.inchesToMeters(77); // FROM THE BOTTOM OF FIXED STAGE TO TOP ELEVATOR

        double MASS_KG = 10.0;
        double DRUM_RADIUS_METERS = (MAX_HEIGHT_METERS / Encoders.NUM_ROTATIONS_TO_REACH_TOP * Encoders.GEARING) / 2 / Math.PI;

        public interface Encoders {
            double GEARING = 4.0;

            double NUM_ROTATIONS_TO_REACH_TOP = (6 + 9.0 / 24) * GEARING; // Number of rotations that the motor has to spin, NOT the gear

            double POSITION_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP;
            double VELOCITY_CONVERSION_FACTOR = MAX_HEIGHT_METERS / NUM_ROTATIONS_TO_REACH_TOP / 60;
        }
    }
}
