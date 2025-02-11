/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    public interface Elevator {
        SparkBaseConfig frontMotor = new SparkMaxConfig().inverted(true).smartCurrentLimit(75).openLoopRampRate(0.75).idleMode(IdleMode.kBrake);
        SparkBaseConfig backMotor = new SparkMaxConfig().inverted(false).smartCurrentLimit(75).openLoopRampRate(0.75).idleMode(IdleMode.kBrake);

        EncoderConfig encoderConfig = new EncoderConfig().positionConversionFactor(Constants.Elevator.Encoders.POSITION_CONVERSION_FACTOR).velocityConversionFactor(Constants.Elevator.Encoders.VELOCITY_CONVERSION_FACTOR);
    }

    public interface Shooter {
        SparkBaseConfig topMotorConfig = new SparkMaxConfig().inverted(false).smartCurrentLimit(50).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
        SparkBaseConfig bottomMotorConfig = new SparkMaxConfig().inverted(true).smartCurrentLimit(50).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
    }

    public interface Funnel {
        SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(false).smartCurrentLimit(50).openLoopRampRate(0.5).idleMode(IdleMode.kBrake);
    }

    public interface Swerve {
        public interface Turn {
            SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(true).smartCurrentLimit(200).openLoopRampRate(0.25).idleMode(IdleMode.kBrake);
        }
        public interface Drive {
            Slot0Configs slot0Configs = new Slot0Configs()
                .withKS(Settings.Swerve.Drive.kS)
                .withKV(Settings.Swerve.Drive.kV)
                .withKA(Settings.Swerve.Drive.kA)
                .withKP(Settings.Swerve.Drive.kP)
                .withKI(Settings.Swerve.Drive.kI)
                .withKD(Settings.Swerve.Drive.kD);
            
            MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

            ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
                .withTorqueClosedLoopRampPeriod(0.25);
            
            CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(65);

            FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(1/Constants.Swerve.Encoder.Drive.POSITION_CONVERSION);

            TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withSlot0(slot0Configs)
                .withMotorOutput(motorOutputConfigs)
                .withClosedLoopRamps(closedLoopRampsConfigs)
                .withCurrentLimits(currentLimitsConfigs)
                .withFeedback(feedbackConfigs);
        }
    }
}
