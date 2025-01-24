/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/*-
 * File containing all of the configurations that different motors require.
 *
 * Such configurations include:
 *  - If it is Inverted
 *  - The Idle Mode of the Motor
 *  - The Current Limit
 *  - The Open Loop Ramp Rate
 */
public interface Motors {
    public interface Algae {
        SparkBaseConfig pivotMotorConfig = new SparkMaxConfig().inverted(false).smartCurrentLimit(50).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
        SparkBaseConfig rollerMotorConfig = new SparkMaxConfig().inverted(false).smartCurrentLimit(50).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
    }

    public interface Elevator {
        SparkBaseConfig leftMotor = new SparkMaxConfig().inverted(false).smartCurrentLimit(100).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
        SparkBaseConfig rightMotor = new SparkMaxConfig().inverted(false).smartCurrentLimit(100).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);

        EncoderConfig encoderConfig = new EncoderConfig().positionConversionFactor(Settings.Elevator.Encoders.POSITION_CONVERSION_FACTOR).velocityConversionFactor(Settings.Elevator.Encoders.VELOCITY_CONVERSION_FACTOR);
    }

    public interface Shooter {
        SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(false).smartCurrentLimit(50).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
    }

    public interface Funnel {
        SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(false).smartCurrentLimit(50).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
    }
}
