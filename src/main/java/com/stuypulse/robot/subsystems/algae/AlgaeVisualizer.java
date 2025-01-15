package com.stuypulse.robot.subsystems.algae;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AlgaeVisualizer {

    public static AlgaeVisualizer instance = new AlgaeVisualizer();  

    private Mechanism2d algae;

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
        algae = new Mechanism2d(100, 100);
        // the vaules here NEED to be checked
        baseRoot = algae.getRoot("base", 50, 50); 
        baseLigament = new MechanismLigament2d("baseLigament", 10, 90);
        
        pivotRoot = algae.getRoot("pivot", 50, 60);
        pivotLigament = new MechanismLigament2d("pivotLigament", 10, 135);

        barRoot = algae.getRoot("bar", 42.83, 67.07);
        barLigament = new MechanismLigament2d("barLigament", 10, 180);
        
        leftRollerRoot = algae.getRoot("leftRoller", 34.5, 67.07);
        leftRollerLigament  = new MechanismLigament2d("leftRollerLigament", 2, 260);
        
        rightRollerRoot = algae.getRoot("rightRoller", 40.5, 67.07);
        rightRollerLigament = new MechanismLigament2d("rightRollerLigament", 2, 280);

        baseRoot.append(baseLigament);
        pivotRoot.append(pivotLigament);
        barRoot.append(barLigament);
        rightRollerRoot.append(rightRollerLigament); 
        leftRollerRoot.append(leftRollerLigament);
}           

public void update(){
    SmartDashboard.putData("algae", algae);
    
}
}