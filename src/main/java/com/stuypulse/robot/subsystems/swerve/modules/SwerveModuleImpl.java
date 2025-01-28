package com.stuypulse.robot.subsystems.swerve.modules;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleImpl extends SwerveModule {

    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final SparkMax pivotMotor;
    private final CANcoder pivotEncoder;

    private final AngleController pivotController;

    public SwerveModuleImpl(String name, Translation2d location, Rotation2d angleOffset, int driveMotorID, int pivotMotorID, int pivotEncoderID) {
        super(name, location);

        this.angleOffset = angleOffset;

        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(Motors.Swerve.Turn.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = new CANcoder(pivotEncoderID);
        
        driveMotor = new TalonFX(driveMotorID);

        driveMotor.getConfigurator().apply(Motors.Swerve.Drive.motorConfig);
        driveMotor.setPosition(0);

        pivotController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);
    }

    public double getPosition() {
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(pivotEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(angleOffset);
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    @Override
    public void periodic() {
        super.periodic();

        pivotController.update(Angle.fromRotation2d(getTargetState().angle), Angle.fromRotation2d(getAngle()));

        if (Math.abs(getTargetState().speedMetersPerSecond) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
            driveMotor.setControl(new VelocityVoltage(0));
            pivotMotor.setVoltage(0);
        } else {
            driveMotor.setControl(new VelocityVoltage(getTargetState().speedMetersPerSecond).withEnableFOC(true));
            pivotMotor.setVoltage(pivotController.getOutput());
        }

        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Current", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Position", getPosition());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Voltage", pivotController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Current", pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Angle Error", pivotController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Raw Encoder Angle", Units.rotationsToDegrees(pivotEncoder.getAbsolutePosition().getValueAsDouble()));
    }
}