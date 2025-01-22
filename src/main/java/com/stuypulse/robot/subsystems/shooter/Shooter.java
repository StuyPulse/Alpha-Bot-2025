package com.stuypulse.robot.subsystems.shooter;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static final Shooter instance;

    static {
        instance = new Shooter();
    }

    public static Shooter getInstance() {
        return instance;
    }

    private final SparkMax shooterMotor;

    public Shooter(){
        super();
        shooterMotor = new SparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
    }

    public void shoot() {
        shooterMotor.set(1);
    }

    public void stop(){
        shooterMotor.set(0);
    }

    @Override
    public void periodic() {
    }
}
