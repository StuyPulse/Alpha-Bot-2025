package com.stuypulse.robot.subsystems.shooter;
import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.databind.deser.impl.FailingDeserializer;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static final Shooter instance;

    static {
        instance = new Shooter();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum ShooterState {
        SHOOTING,
        STOP
    }

    private ShooterState shooterState;

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final CANcoder leftEncoder;
    private final CANcoder rightEncoder;
 
    private final PIDController leftController;
    private final PIDController rightController;

    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;

    public Shooter(){
        super();

        leftMotor = new SparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new SparkMax(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);

        leftEncoder = new CANcoder(Ports.Shooter.LEFT_ENCODER);
        rightEncoder = new CANcoder(Ports.Shooter.RIGHT_ENCODER);

        leftController = new PIDController(Settings.Shooter.LEFT_kP, Settings.Shooter.LEFT_kI, Settings.Shooter.LEFT_kD);
        rightController = new PIDController(Settings.Shooter.RIGHT_kP, Settings.Shooter.RIGHT_kI, Settings.Shooter.RIGHT_kD);

        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", Settings.Shooter.LEFT_TARGET_RPM);
        rightTargetRPM = new SmartNumber("Shooter/Right Target RMP", Settings.Shooter.RIGHT_TARGET_RPM);
    }
    
    private double getLeftShooterRPM() {
        return leftEncoder.getVelocity().getValueAsDouble();
    }

    private double getRightShooterRPM() {
        return rightEncoder.getVelocity().getValueAsDouble();
    }

    public boolean atTargetSpeeds() {
        return Math.abs(getLeftShooterRPM() - leftTargetRPM.get()) < Settings.Shooter.TARGET_EPSILON
            && Math.abs(getRightShooterRPM() - rightTargetRPM.get()) < Settings.Shooter.TARGET_EPSILON;
    }

    private void setTargetSpeeds(double leftRPM, double rightRPM) {
        this.leftTargetRPM.set(leftRPM);
        this.rightTargetRPM.set(rightRPM);
    }

    private void setLeftShooterSpeed(double rpm) {
        leftMotor.set(rpm);
    }

    private void setRightShooterRPM(double rpm) {
        rightMotor.set(rpm);
    }

    // public void stop(){
    //     leftMotor.stopMotor();
    //     rightMotor.stopMotor();
    // }

    // public void shoot(){
    //     leftMotor.set(Settings.Shooter.LEFT_SHOOT_SPEED_L1);
    //     rightMotor.set(Settings.Shooter.RIGHT_SHOOT_SPEED_L1);
    // }

    @Override
    public void periodic() {
        switch(getShooterState()) {
            case SHOOTING:
                setTargetSpeeds(leftTargetRPM.getAsDouble(), rightTargetRPM.getAsDouble());
                break;
            case STOP:
                setTargetSpeeds(0, 0);
                break;
            default:
                setTargetSpeeds(0, 0);
                break;
        }

        if(leftTargetRPM.get() == 0) {
            leftMotor.set(0);
        } else {
            setLeftShooterSpeed(leftTargetRPM.get());
        }

        if(rightTargetRPM.get() == 0) {
            rightMotor.set(0);
        } else {
            setRightShooterRPM(rightTargetRPM.get());
        }

        SmartDashboard.putString("Shooter State", getShooterState().toString());

        SmartDashboard.putNumber("Shooter/Left Motor RPM", getLeftShooterRPM());
        SmartDashboard.putNumber("Shooter/Right Motor RPM", getRightShooterRPM());

        SmartDashboard.putNumber("Shooter/Left Target RPM", leftTargetRPM.get());
        SmartDashboard.putNumber("Shooter/Right Target RPM", rightTargetRPM.get());
    
        SmartDashboard.putBoolean("Shooter/At Target Speeds", atTargetSpeeds());
    }
}
