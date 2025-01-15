package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.units.measure.Angle;

public class SwerveModuleImpl extends SwerveModule{

   
    private final String name;
    private final Translation2d location;

    private SwerveModuleState targetState;

    // turn

    private final SparkMax turnMotor;
    private final CANcoder absoluteEncoder;
    private final Angle angleOffset;

    private final AngleController turnController;

    // drive
    private final TalonFX driveMotor;
    private final RelativeEncoder driveEncoder;

    private final Controller driveController;

   public SwerveModuleImpl(String name, int driveID, int turnID, int canCoderID, Translation2d location, Angle angleOffset, boolean driveInverted, boolean turnInverted) {
        this.name = name;
        this.location = location;
        this.angleOffset = angleOffset;

        driveMotor = new TalonFX(driveID);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

         driveConfig.Slot0 = Slot0

        // Direction and neutral mode
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Ramp rates
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.0; // 100ms

        // Gear ratio
        driveConfig.Feedback.SensorToMechanismRatio = 1.0; // 1:1 sensor to mechanism ratio

        // Current limits
        driveConfig.CurrentLimits.StatorCurrentLimit = 65; // 65A stator current limit
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true; // Enable stator current limiting


        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = +400; // 40A peak forward torque current
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -400; // 40A peak reverse torque current
        driveConfig.TorqueCurrent.TorqueNeutralDeadband = 0.05; // 5% torque neutral deadband

        driveEncoder = driveMotor.getEncoder();
    
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        SparkBaseConfig turnConfig = new SparkMaxConfig().inverted(turnInverted).idleMode(IdleMode.kBrake);
        turnConfig.encoder.positionConversionFactor(.Turn.POSITION_CONVERSION).velocityConversionFactor(`.Turn.VELOCITY_CONVERSION);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.driveController = new 
        (Drive.kP, Drive.kP, Drive.kD)
                                .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
        this.turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD);

        absoluteEncoder = new CANcoder(canCoderID);

        targetState = new SwerveModuleState();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getName'");
    }

    @Override
    public Translation2d getOffset() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getOffset'");
    }

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getModulePosition'");
    }

    @Override
    public void setTargetState(SwerveModuleState swerveModuleState) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTargetState'");
    }
    
}
