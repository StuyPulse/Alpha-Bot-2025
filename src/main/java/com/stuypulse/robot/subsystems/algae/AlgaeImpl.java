package com.stuypulse.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ArmFeedForward;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.numbers.filters.MotionProfile;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeImpl extends Algae {

    private SparkMax pivotMotor;
    private SparkMax rollerMotor;
    private CANcoder pivotEncoder;

    private Controller pivotController;

    private SmartNumber targetAngle;

    private final AlgaeVisualizer visualizer;

    public AlgaeImpl() {
        pivotMotor = new SparkMax(com.stuypulse.robot.constants.Ports.Algae.PIVOT, MotorType.kBrushless);
        pivotMotor.configure(Motors.Algae.pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        rollerMotor = new SparkMax(com.stuypulse.robot.constants.Ports.Algae.ROLLER, MotorType.kBrushless);
        rollerMotor.configure(Motors.Algae.rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotEncoder = new CANcoder(Ports.Algae.ENCODER);

        MotionProfile motionProfile = new MotionProfile(Settings.Algae.MAX_ANGULAR_VELOCITY_RAD_PER_SECOND, Settings.Algae.MAX_ANGULAR_ACCEL_RAD_PER_SECOND_PER_SECOND);
        
        pivotController = new MotorFeedforward(Settings.Algae.FF.kS, Settings.Algae.FF.kV, Settings.Algae.FF.kA).position()
            .add(new ArmFeedForward(Settings.Algae.FF.kG))
            .add(new PIDController(Settings.Algae.PID.kP, Settings.Algae.PID.kI, Settings.Algae.PID.kD))
            .setSetpointFilter(motionProfile);
        
        targetAngle = new SmartNumber("Algae Mech/Target Angle", 0); 

        visualizer = new AlgaeVisualizer();
    }

    @Override
    public double getTargetAngle() {
        return targetAngle.get();
    }

    @Override
    public double getCurrentAngle() {
        return Units.rotationsToDegrees(pivotEncoder.getPosition().getValueAsDouble()) - Settings.Algae.ENCODER_OFFSET_DEGREES;
    }

    @Override
    public void setTargetAngle(double angle) {
        this.targetAngle.set(angle);
    }

    @Override
    public void acquireUnder() {
        rollerMotor.set(Settings.Algae.ACQUIRE_SPEED.get());
    }

    @Override
    public void acquireOver() {
        rollerMotor.set(-Settings.Algae.ACQUIRE_SPEED.get());
    }

    @Override
    public void deacquireUnder() {
        rollerMotor.set(-Settings.Algae.DEACQUIRE_SPEED.get());
    }
    
    @Override
    public void deacquireOver() {
        rollerMotor.set(Settings.Algae.DEACQUIRE_SPEED.get());
    }

    @Override
    public void stopRollers() {
        rollerMotor.set(0);
    }

    @Override
    public boolean hasAlgae() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'hasAlgae'");
    }

    @Override
    public void periodic() {
        pivotController.update(getTargetAngle(), getCurrentAngle());
        pivotMotor.setVoltage(pivotController.getOutput());
        
        visualizer.update();
        visualizer.updateBarAngle();
        visualizer.updatePivotAngle();

        // SmartDashboard.putBoolean("Algae Mech/Has Algae", hasAlgae());
    }
}