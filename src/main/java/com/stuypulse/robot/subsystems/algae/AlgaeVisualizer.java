package com.stuypulse.robot.subsystems.algae;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AlgaeVisualizer {

    public static AlgaeVisualizer instance = new AlgaeVisualizer();  

    private Mechanism2d algaeVis;
    private Algae algae;

    // base declarations
    private MechanismRoot2d baseRoot;
    private MechanismLigament2d baseLigament;
    
    // pivot declarations
    private MechanismRoot2d pivotRoot;
    private MechanismLigament2d pivotLigament;

    // bar declarations
    private MechanismRoot2d barRoot;
    private MechanismLigament2d barLigament;
    
    // roller declarations (2 rollers) - 4 declarations in this section
    private MechanismRoot2d leftRollerRoot;
    private MechanismLigament2d leftRollerLigament;
    
    private MechanismRoot2d rightRollerRoot;
    private MechanismLigament2d rightRollerLigament;
    
    public AlgaeVisualizer() {
        algae = Algae.getInstance();

        algaeVis = new Mechanism2d(100, 100);
        baseRoot = algaeVis.getRoot("base", 50, 50); 
        baseLigament = new MechanismLigament2d("baseLigament", 10, 90);
        
        pivotRoot = algaeVis.getRoot("pivot", 50, 60);
        pivotLigament = new MechanismLigament2d("pivotLigament", 10, 135);

        barRoot = algaeVis.getRoot("bar", 42.93, 67.07);
        barLigament = new MechanismLigament2d("barLigament", 10, 180);
        
        leftRollerRoot = algaeVis.getRoot("leftRoller", 34.5, 67.07);
        leftRollerLigament  = new MechanismLigament2d("leftRollerLigament", 1, 260);
        
        rightRollerRoot = algaeVis.getRoot("rightRoller", 40.5, 67.07);
        rightRollerLigament = new MechanismLigament2d("rightRollerLigament", 1, 280);

        baseRoot.append(baseLigament);
        pivotRoot.append(pivotLigament);
        barRoot.append(barLigament);
        rightRollerRoot.append(rightRollerLigament); 
        leftRollerRoot.append(leftRollerLigament);
    }

    public void spinVisualizerRollersIntake() {
        rightRollerLigament.setAngle(rightRollerLigament.getAngle() - 5);
        leftRollerLigament.setAngle(leftRollerLigament.getAngle() + 5);
    }

    public void spinVisualizerRollersOutake() {
        rightRollerLigament.setAngle(rightRollerLigament.getAngle() + 5);
        leftRollerLigament.setAngle(leftRollerLigament.getAngle() - 5);
    }

    public void updatePivotAngle() {
        //pivotLigament.setAngle(Algae.getInstance().getCurrentAngle());
        pivotLigament.setAngle(pivotLigament.getAngle()+1);
    }

    public void updateBarAngle(){
        //barLigament.setAngle(Algae.getInstance().getCurrentAngle() + 45);
        //aligns bar rotation and position to pivot ligament
        barLigament.setAngle(pivotLigament.getAngle()+45); 
        barRoot.setPosition(
            pivotLigament.getLength() * Math.cos(Math.toRadians(pivotLigament.getAngle())) + 50,
            pivotLigament.getLength() * Math.sin(Math.toRadians(pivotLigament.getAngle())) + 60
            );
        }

    public void updateRollerPositions() {
        //rightRollerLigament.setAngle(Algae.getInstance());
        //aligns right roller to bar
        rightRollerRoot.setPosition(
            -2.43 * Math.cos(Math.toRadians(pivotLigament.getAngle()-135)) + pivotLigament.getLength() * Math.cos(Math.toRadians(pivotLigament.getAngle())) + 50,
            -2.43 * Math.sin(Math.toRadians(pivotLigament.getAngle()-135)) + pivotLigament.getLength() * Math.sin(Math.toRadians(pivotLigament.getAngle()))+ 60
        );
        // aligns left roller to bar 
        leftRollerRoot.setPosition(
            -8.43 * Math.cos(Math.toRadians(pivotLigament.getAngle()-135)) + pivotLigament.getLength() * Math.cos(Math.toRadians(pivotLigament.getAngle())) + 50,
            -8.43 * Math.sin(Math.toRadians(pivotLigament.getAngle()-135)) + pivotLigament.getLength() * Math.sin(Math.toRadians(pivotLigament.getAngle()))+ 60
        );
    }
// daniel ❤️ diddy

    public void update() {
        SmartDashboard.putData("algae", algaeVis);
    }
}