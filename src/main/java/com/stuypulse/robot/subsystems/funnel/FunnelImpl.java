package com.stuypulse.robot.subsystems.funnel;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FunnelImpl extends Funnel{
    
    private SparkMax motor;

    public FunnelImpl() {
        motor = new SparkMax(Ports.Funnel.MOTOR, MotorType.kBrushless);
        motor.configure(Motors.Funnel.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void acquire() {
        motor.set(Settings.Funnel.ACQUIRE_SPEED.get());
    }

    @Override
    public void deacquire() {
        motor.set(-Settings.Funnel.DEACQUIRE_SPEED.get());
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Funnel/Speed", motor.get());
    }
}
