package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter{
    
    private final SparkMax motor;
    private final DigitalInput IR_Sensor;

    public ShooterImpl() {
        motor = new SparkMax(Ports.Shooter.MOTOR, MotorType.kBrushless);
        motor.configure(Motors.Shooter.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IR_Sensor = new DigitalInput(Ports.Shooter.IR_SENSOR);
    }

    @Override
    public void acquire() {
        motor.set(Settings.Shooter.ACQUIRE_SPEED.get());
    }

    @Override
    public void shoot() {
        motor.set(Settings.Shooter.SHOOT_SPEED.get());
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public boolean hasCoral() {
        return IR_Sensor.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/Has Coral", hasCoral());
    }
}
