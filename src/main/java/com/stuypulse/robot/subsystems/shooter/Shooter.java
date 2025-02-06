package com.stuypulse.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {

    private static final Shooter instance;

    static {
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public abstract void acquire();
    public abstract void shoot();
    public abstract void stop();

    public abstract boolean hasCoral();
}
