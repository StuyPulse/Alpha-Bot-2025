package com.stuypulse.robot.subsystems.algae;

import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Algae extends SubsystemBase {

    public static final Algae instance;

    static {
        instance = new AlgaeImpl();
    }

    public Algae() {

    }

    public static Algae getInstance() {
        return instance;
    }
    
    public abstract double getTargetAngle();

    public abstract double getCurrentAngle();

    public abstract void acquireUnder();

    public abstract void acquireOver();

    public abstract void deacquireUnder();

    public abstract void deacquireOver();
    
    public abstract void stopRollers();

    public abstract void setTargetAngle(double targetAngle);
}

