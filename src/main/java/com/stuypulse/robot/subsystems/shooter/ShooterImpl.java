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
    
    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    private final DigitalInput IR_Sensor;

    public ShooterImpl() {
        topMotor = new SparkMax(Ports.Shooter.TOP_MOTOR, MotorType.kBrushless);
        topMotor.configure(Motors.Shooter.topMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        bottomMotor = new SparkMax(Ports.Shooter.BOTTOM_MOTOR, MotorType.kBrushless);
        bottomMotor.configure(Motors.Shooter.bottomMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        IR_Sensor = new DigitalInput(Ports.Shooter.IR_SENSOR);
    }

    @Override
    public void acquire() {
        topMotor.set(Settings.Shooter.Top.ACQUIRE_SPEED.get());
        bottomMotor.set(Settings.Shooter.Bottom.ACQUIRE_SPEED.get());
    }

    @Override
    public void shoot() {
        topMotor.set(Settings.Shooter.Top.SHOOT_SPEED.get());
        bottomMotor.set(Settings.Shooter.Bottom.SHOOT_SPEED.get());
    }

    @Override
    public void stop() {
        topMotor.set(0);
        bottomMotor.set(0);
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
