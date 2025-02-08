package com.stuypulse.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.stuypulse.robot.constants.Constants;
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
    private final SparkMax frontMotor;
    private final SparkMax backMotor;

    private final RelativeEncoder encoder;

    private final SmartNumber targetHeight;

    private final Controller controller;

    private boolean hasBeenReset;

    public ElevatorImpl() {
        frontMotor = new SparkMax(Ports.Elevator.FRONT, MotorType.kBrushless);
        Motors.Elevator.frontMotor.encoder.apply(Motors.Elevator.encoderConfig);
        frontMotor.configure(Motors.Elevator.frontMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        backMotor = new SparkMax(Ports.Elevator.BACK, MotorType.kBrushless);
        Motors.Elevator.backMotor.encoder.apply(Motors.Elevator.encoderConfig);
        backMotor.configure(Motors.Elevator.backMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = frontMotor.getEncoder();
        encoder.setPosition(Constants.Elevator.MIN_HEIGHT_METERS);

        targetHeight = new SmartNumber("Elevator/Target Height", Constants.Elevator.MIN_HEIGHT_METERS);

        MotionProfile motionProfile = new MotionProfile(Settings.Elevator.MAX_VELOCITY_METERS_PER_SECOND, Settings.Elevator.MAX_ACCEL_METERS_PER_SECOND_PER_SECOND);
        
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
                Constants.Elevator.MIN_HEIGHT_METERS, 
                Constants.Elevator.MAX_HEIGHT_METERS
            )
        );
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }

    @Override
    public double getCurrentHeight() {
        return encoder.getPosition();
    }

    @Override
    public boolean atTargetHeight() {
        return Math.abs(getTargetHeight() - getCurrentHeight()) < Settings.Elevator.HEIGHT_TOLERANCE_METERS.get();
    }

    private void setVoltage(double voltage) {
        frontMotor.setVoltage(voltage);
        backMotor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!hasBeenReset) {
            setVoltage(-3.0);
            if (frontMotor.getOutputCurrent() > Settings.Elevator.RESET_STALL_CURRENT || backMotor.getOutputCurrent() > Settings.Elevator.RESET_STALL_CURRENT) {
                hasBeenReset = true;
                encoder.setPosition(Constants.Elevator.MIN_HEIGHT_METERS);
            }
        }
        else {
            controller.update(getTargetHeight(), getCurrentHeight());
            setVoltage(controller.getOutput());
        }

        SmartDashboard.putNumber("Elevator/Current Height", getCurrentHeight());

        SmartDashboard.putNumber("Elevator/Front current", frontMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Back current", backMotor.getOutputCurrent());
    }
}