package com.stuypulse.robot.subsystems.algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeImpl extends Algae {

        // variable declaration
        

    private SparkMax pivotMotor;
    private SparkMax rollerMotor;

    private RelativeEncoder pivotEncoder;
    private RelativeEncoder rollerEncoder;

    private PIDController pivotPIDController;
    private ArmFeedforward pivotFFController;

    private double targetAngle;
    //private int rollerState; // -1 is deacquire, 1 is acquire/intake, 0 is not moving
    
        // constructor

    public AlgaeImpl() {
        pivotMotor = new SparkMax(Settings.Algae.PIVOT_ID, MotorType.kBrushless);
        rollerMotor = new SparkMax(Settings.Algae.ROLLER_ID, MotorType.kBrushless);
        
        pivotEncoder = pivotMotor.getEncoder();
        rollerEncoder = rollerMotor.getEncoder();
        
        pivotPIDController = new PIDController(Settings.Algae.PID.kP, Settings.Algae.PID.kI, Settings.Algae.PID.kD);
        pivotFFController = new ArmFeedforward(Settings.Algae.FF.kS, Settings.Algae.FF.kG, Settings.Algae.FF.kV, Settings.Algae.FF.kA);
        targetAngle = Settings.Algae.TARGET_ANGLE; // abitrary target angle - change in settings

    }

    // pivot
    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentAngle() {
        return Units.rotationsToDegrees(pivotEncoder.getPosition());
    }

    public double getCurrentPivotVelocity() {
        return pivotEncoder.getVelocity();
    }



        // setters
    
    // rollers
    public void acquire() { // only rollers
        rollerMotor.set(1);
    }

    public void deacquire() { // only rollers
        rollerMotor.set(-1);
    }

    public void stopRollers() {
        rollerMotor.set(0);
    }
    
    // pivot

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }


    // periodic

    @Override
    public void periodic() { // check pararmeter types for our Controllers, probably FF
        // --------------- CHECK EVERYTHING ---------------
        pivotPIDController.calculate(getCurrentAngle(), getTargetAngle());
        pivotFFController.calculate(Math.toRadians(getTargetAngle()), getCurrentPivotVelocity()); 
        
        
    }

}