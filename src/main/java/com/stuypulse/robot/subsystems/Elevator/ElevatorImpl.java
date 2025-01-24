package com.stuypulse.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorImpl extends Elevator {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final DigitalInput bumpSwitch;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SmartNumber targetHeight;

    private final Controller controller;

    private boolean hasBeenReset;

    public ElevatorImpl() {
        leftMotor = new SparkMax(Ports.Elevator.LEFT, MotorType.kBrushless);
        Motors.Elevator.leftMotor.encoder.apply(Motors.Elevator.encoderConfig);
        leftMotor.configure(Motors.Elevator.leftMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightMotor = new SparkMax(Ports.Elevator.RIGHT, MotorType.kBrushless);
        Motors.Elevator.rightMotor.encoder.apply(Motors.Elevator.encoderConfig);
        rightMotor.configure(Motors.Elevator.rightMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        bumpSwitch = new DigitalInput(Ports.Elevator.SWITCH);

        targetHeight = new SmartNumber("Elevator/Target Height", Settings.Elevator.MIN_HEIGHT_METERS);

        MotionProfile motionProfile = new MotionProfile(Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND, Settings.Elevator.MAX_ACCELERATION_METERS_PER_SECOND);
        
        controller = new MotorFeedforward(Settings.Elevator.FF.kS, Settings.Elevator.FF.kV, Settings.Elevator.FF.kA).position()
            .add(new ElevatorFeedforward(Settings.Elevator.FF.kG))
            .add(new PIDController(Settings.Elevator.PID.kP, Settings.Elevator.PID.kI, Settings.Elevator.PID.kD))
            .setSetpointFilter(motionProfile);
        
        hasBeenReset = false;
    }
    
    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(
            SLMath.clamp(
                height, 
                Settings.Elevator.MIN_HEIGHT_METERS, 
                Settings.Elevator.MAX_HEIGHT_METERS
            )
        );
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }

    @Override
    public double getCurrentHeight() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    private void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (bumpSwitch.get()) {
            hasBeenReset = true;
            leftMotor.getEncoder().setPosition(Settings.Elevator.MIN_HEIGHT_METERS);
            rightMotor.getEncoder().setPosition(Settings.Elevator.MIN_HEIGHT_METERS);
        }

        if (!hasBeenReset) {
            setVoltage(-1);
        }
        else {
            controller.update(getTargetHeight(), getCurrentHeight());
            setVoltage(controller.getOutput());
        }

        SmartDashboard.putNumber("Elevator/Target Height", targetHeight.getAsDouble());
        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());
    }
}