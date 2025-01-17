package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve;
import com.stuypulse.robot.constants.Settings.Swerve.Encoder;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.numbers.filters.Derivative;
import com.stuypulse.stuylib.streams.numbers.filters.IFilter;
import com.stuypulse.stuylib.streams.numbers.filters.TimedMovingAverage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleImpl extends SwerveModule {

    public static SwerveModule[] getTumblerModules() {
        return new SwerveModule[] {
            new SwerveModuleImpl("Front Right", Swerve.FrontRight.MODULE_OFFSET, Rotation2d.fromDegrees(-153.632812 + 180), 12, 15, 4),
            new SwerveModuleImpl("Front Left",  Swerve.FrontLeft.MODULE_OFFSET,  Rotation2d.fromDegrees(147.919922 + 180),  10, 17, 2),
            new SwerveModuleImpl("Back Left",   Swerve.BackLeft.MODULE_OFFSET,   Rotation2d.fromDegrees(73.125 + 180),      14, 11, 3),
            new SwerveModuleImpl("Back Right",  Swerve.BackRight.MODULE_OFFSET,  Rotation2d.fromDegrees(-2.02184 + 180),    16, 13, 1)
        };
    }

    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final SparkMax pivotMotor;
    private final CANcoder pivotEncoder;

    private final AngleController pivotController;

    private final IFilter targetAcceleration;

    public SwerveModuleImpl(
        String id, 
        Translation2d location, 
        Rotation2d angleOffset, 
        int driveMotorID, 
        int pivotMotorID, 
        int pivotEncoderID
    ) {
        super(id, location);

        this.angleOffset = angleOffset;

        driveMotor = new TalonFX(driveMotorID);
        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        pivotEncoder = new CANcoder(pivotEncoderID);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // PIDF values
        Slot0Configs slot0 = new Slot0Configs();

        slot0.kS = 0.14304; 
        slot0.kV = 0.10884; 
        slot0.kA = 0.023145; 
        slot0.kP = 0.07; 
        slot0.kI = 0; 
        slot0.kD = 0; 

        driveConfig.Slot0 = slot0;

        // Direction and neutral mode
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Ramp rates
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.0; // 100ms

        driveConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1 sensor to mechanism ratio

        // Current limits
        driveConfig.CurrentLimits.StatorCurrentLimit = 65; // 65A stator current limit
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Enable stator current limiting

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = +400; // 40A peak forward torque current
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -400; // 40A peak reverse torque current
        driveConfig.TorqueCurrent.TorqueNeutralDeadband = 0.05; // 5% torque neutral deadband


        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setPosition(0);

        pivotController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
            .setOutputFilter(x -> -x);

        targetAcceleration = new Derivative()
            .then(new TimedMovingAverage(0.1))
            .then(x -> MathUtil.clamp(x, 0, Settings.Swerve.MAX_MODULE_ACCEL));

            SparkBaseConfig turnConfig = new SparkMaxConfig().inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(80);
            turnConfig.encoder.positionConversionFactor(Swerve.Encoder.Turn.POSITION_CONVERSION).velocityConversionFactor(Swerve.Encoder.Drive.VELOCITY_CONVERSION);
            pivotMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getPosition() {
        return driveMotor.getPosition().getValueAsDouble() * Encoder.Drive.POSITION_CONVERSION;
    }

    @Override
    public double getVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * Encoder.Drive.POSITION_CONVERSION;
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

    private double convertDriveVel(double speedMetersPerSecond) {
        return speedMetersPerSecond / Encoder.Drive.POSITION_CONVERSION;
    }

    @Override
    public void periodic() {
        super.periodic();

        final boolean USE_ACCEL = true;
        final boolean USE_ACCEL_IN_AUTON = true;
        final boolean USE_FOC_IN_AUTON = true;

        double velocity = convertDriveVel(getTargetState().speedMetersPerSecond);
        double acceleration = targetAcceleration.get(velocity);
        boolean useFOC = true;

        if (!USE_ACCEL) {
            acceleration = 0;
        }

        if (DriverStation.isAutonomousEnabled()) {
            if (!USE_ACCEL_IN_AUTON) {
                acceleration = 0;
            }

            if (!USE_FOC_IN_AUTON) {
                useFOC = false;
            }
        }

        VelocityVoltage driveOutput = new VelocityVoltage(velocity)
            .withAcceleration(acceleration)
            .withEnableFOC(useFOC);

        pivotController.update(Angle.fromRotation2d(getTargetState().angle), Angle.fromRotation2d(getAngle()));

        if (Math.abs(getTargetState().speedMetersPerSecond) < Settings.Swerve.MODULE_VELOCITY_DEADBAND) {
            driveMotor.setControl(new VelocityVoltage(0));
            pivotMotor.setVoltage(0);
        } else {
            driveMotor.setControl(driveOutput);
            pivotMotor.setVoltage(pivotController.getOutput());
        }

        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Target Acceleration", acceleration * Encoder.Drive.POSITION_CONVERSION);
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Current", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Position", getPosition());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Velocity", getVelocity());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Drive Voltage", driveMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Voltage", pivotController.getOutput());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Turn Current", pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Angle Error", pivotController.getError().toDegrees());
        SmartDashboard.putNumber("Swerve/Modules/" + getId() + "/Raw Encoder Angle", Units.rotationsToDegrees(pivotEncoder.getAbsolutePosition().getValueAsDouble()));
    }
    
}