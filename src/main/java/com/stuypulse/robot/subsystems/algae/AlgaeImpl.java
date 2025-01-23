package com.stuypulse.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AlgaeImpl extends Algae {

        // variable declaration
        

    private SparkMax pivotMotor;
    private SparkMax rollerMotor;

    private RelativeEncoder pivotEncoder;

    private ProfiledPIDController pivotPIDController;
    private ArmFeedforward pivotFFController;

    private final AlgaeVisualizer visualizer;

    private double targetAngle;

    private Constraints constraints;

    //private int rollerState; // -1 is deacquire, 1 is acquire/intake, 0 is not moving
    
        // constructor

    public AlgaeImpl() {
        pivotMotor = new SparkMax(com.stuypulse.robot.constants.Ports.Algae.PIVOT_ID, MotorType.kBrushless);
        rollerMotor = new SparkMax(com.stuypulse.robot.constants.Ports.Algae.ROLLER_ID, MotorType.kBrushless);

        
        pivotEncoder = pivotMotor.getEncoder();

        constraints = new Constraints(Settings.Algae.PID.MAX_VELOCITY, Settings.Algae.PID.MAX_ACCELERATION);
        
        pivotPIDController = new ProfiledPIDController(Settings.Algae.PID.kP, Settings.Algae.PID.kI, Settings.Algae.PID.kD, constraints);
        pivotFFController = new ArmFeedforward(Settings.Algae.FF.kS, Settings.Algae.FF.kG, Settings.Algae.FF.kV, Settings.Algae.FF.kA);
        targetAngle = 0; 

        visualizer = new AlgaeVisualizer();
    }

    // pivot
    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        // return Units.rotationsToDegrees(pivotEncoder.getPosition()); 
        return 135;
    }

        // setters
    
    // rollers
    public void acquireUnder() { // only rollers - GROUND AND L2
        rollerMotor.set(Settings.Algae.ACQUIRE_SPEED);
    }

    public void acquireOver() { // only rollers - L3
        rollerMotor.set(-Settings.Algae.ACQUIRE_SPEED);
        // not sure if this should be negative or positive yet
        // but it shoudl be opposite of Under
        // Do we rename acquire speed to smth else
        // Should probably rename aquire speed to smth else
    }

    public void deacquireUnder() { // only rollers - GROUND AND L2
        rollerMotor.set(Settings.Algae.DEACQUIRE_SPEED);
    }
    
    public void deacquireOver() { // L3
        rollerMotor.set(-Settings.Algae.DEACQUIRE_SPEED);
            // not sure if this should be negative or positive yet
            // but it should be opposite of Under
    }

    public void stopRollers() {
        rollerMotor.set(0);
    }
    
    // pivot

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    private double getCurrentPivotVelocity(){
        return pivotEncoder.getVelocity();
    }

    // periodic

    @Override
    public void periodic() {

        pivotMotor.setVoltage(
            pivotPIDController.calculate(
                getCurrentAngle(),
                getTargetAngle()
            ) 
            + 
            pivotFFController.calculate(
                Math.toRadians(getTargetAngle()),
                getCurrentPivotVelocity()
            )
        );
        

        // visualizer updating stuff 

        visualizer.updateBarAngle();
        visualizer.updatePivotAngle(); 
        visualizer.updateRollerPositions();      
        
        visualizer.update();  
    }
}