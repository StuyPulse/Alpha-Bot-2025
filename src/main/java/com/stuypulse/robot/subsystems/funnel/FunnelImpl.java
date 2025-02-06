package com.stuypulse.robot.subsystems.funnel;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FunnelImpl extends Funnel{
    
    private SparkMax motor;
    private BStream shouldReverse;

    public FunnelImpl() {
        motor = new SparkMax(Ports.Funnel.MOTOR, MotorType.kBrushless);
        motor.configure(Motors.Funnel.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shouldReverse = BStream.create(() -> motor.getOutputCurrent() > Settings.Funnel.STALL_CURRENT)
            .filtered(new BDebounce.Rising(Settings.Funnel.STALL_DETECTION_TIME))
            .filtered(new BDebounce.Falling(Settings.Funnel.REVERSE_TIME));
    }

    @Override
    public void acquire() {
        motor.set(Settings.Funnel.ACQUIRE_SPEED.get());
    }

    @Override
    public void reverse() {
        motor.set(-Settings.Funnel.REVERSE_SPEED.get());
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public boolean shouldReverse() {
        return this.shouldReverse.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Funnel/Speed", motor.get());
    }
}
