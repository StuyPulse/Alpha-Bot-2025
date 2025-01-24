package com.stuypulse.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Funnel extends SubsystemBase{
    
    private static final Funnel instance;

    static {
        instance = new FunnelImpl();
    }

    public static Funnel getInstance() {
        return instance;
    }

    public abstract void acquire();
    public abstract void deacquire();
    public abstract void stop();
}
