package com.stuypulse.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorImpl extends Elevator {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final DigitalInput bumpSwitch;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SmartNumber targetHeight;

    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;

    private ElevatorFeedforward FF;
    private ProfiledPIDController PID;

    public ElevatorImpl() {
        leftMotor = new SparkMax(Ports.Elevator.LEFT, MotorType.kBrushless);
        rightMotor = new SparkMax(Ports.Elevator.RIGHT, MotorType.kBrushless);

        rightConfig = new SparkMaxConfig();
        leftConfig = new SparkMaxConfig();

        bumpSwitch = new DigitalInput(Ports.Elevator.SWITCH);

        targetHeight = new SmartNumber("Elevator/Target Height", 0);

        rightConfig.encoder.positionConversionFactor(Settings.Elevator.POSITION_CONVERSION_FACTOR);
        leftConfig.encoder.positionConversionFactor(Settings.Elevator.POSITION_CONVERSION_FACTOR);
        rightConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        rightMotor.configure(leftConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        leftMotor.configure(rightConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

         FF = new ElevatorFeedforward(
            Settings.Elevator.FF.kS.getAsDouble(), 
            Settings.Elevator.FF.kG.getAsDouble(), 
            Settings.Elevator.FF.kV.getAsDouble()
        );

        PID = new ProfiledPIDController(
            Settings.Elevator.PID.kP.getAsDouble(), 
            Settings.Elevator.PID.kI.getAsDouble(), 
            Settings.Elevator.PID.kD.getAsDouble(), 
            new TrapezoidProfile.Constraints(
                Settings.Elevator.MAX_ACCELERATION, 
                Settings.Elevator.MAX_VELOCITY
            )
        );
    }
    
    @Override
    public void setTargetHeight(double height) {
        targetHeight.set(
            SLMath.clamp(
                height, 
                Settings.Elevator.MIN_HEIGHT, 
                Settings.Elevator.MAX_HEIGHT
            )
        );
    }

    @Override
    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }

    @Override
    public double getHeight() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    @Override
    public void stopElevator() {
        rightMotor.stopMotor();
        leftMotor.stopMotor(); 
    }
    @Override
    public boolean atBottom() {
        return !bumpSwitch.get();
    }
    
    public void reset() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        final double PIDOutput = PID.calculate(getHeight(), targetHeight.doubleValue());
        final double FFOutput = FF.calculate(PID.getSetpoint().velocity);

        if (atBottom() && (PIDOutput + FFOutput) < 0) {
            stopElevator();
            reset();
        } else {
            leftMotor.setVoltage(PIDOutput + FFOutput);
            rightMotor.setVoltage(PIDOutput + FFOutput);
        }

        SmartDashboard.putNumber("Elevator/Target Height", targetHeight.getAsDouble());
        SmartDashboard.putNumber("Elevator/Height", getHeight());
    }
}