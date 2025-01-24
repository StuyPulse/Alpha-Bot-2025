/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    double LENGTH = Units.inchesToMeters(29);
    
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

    public interface Swerve {
        // between wheel centers
        double WIDTH = Units.inchesToMeters(20.75);
        double LENGTH = Units.inchesToMeters(20.75);
        double CENTER_TO_INTAKE_FRONT = Units.inchesToMeters(13.0);

        double MAX_MODULE_SPEED = 4.9;
        double MAX_MODULE_ACCEL = 15.0;

        double MODULE_VELOCITY_DEADBAND = 0.05;

        SmartNumber ALIGN_OMEGA_DEADBAND = new SmartNumber("Swerve/Align Omega Deadband", 0.05); // TODO: make 0.25 and test

        public interface Assist {
            SmartNumber ALIGN_MIN_SPEAKER_DIST = new SmartNumber("SwerveAssist/Minimum Distance to Speaker", 4); //change

            double BUZZ_INTENSITY = 1;

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
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
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
            SmartNumber kP = new SmartNumber("Swerve/Drive/PID/kP", 0.31399);
            double kI = 0.0;
            double kD = 0.0;

            double kS = 0.15618;
            SmartNumber kV = new SmartNumber("Swerve/Drive/FF/kV", 0.00012288);
            SmartNumber kA = new SmartNumber("Swerve/Drive/FF/kA", 0.0000259);
        }

        public interface FrontRight {
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(38.144531);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(-173.408203);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(24.609375);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(38.232422);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
        }
    }

    public interface Vision {        
        Vector<N3> STDDEVS = VecBuilder.fill(0.3, 0.3, Math.toRadians(30));
    }

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
