package com.stuypulse.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.feedforward.ElevatorFeedforward;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorVisualizer {

    private static final ElevatorVisualizer instance;

    static {
        instance = new ElevatorVisualizer();
    }

    private final Mechanism2d elevator2d;

    private final MechanismRoot2d elevatorBL;
    private final MechanismRoot2d elevatorTR;

    private final MechanismRoot2d outerBL;
    private final MechanismRoot2d outerTR;

    private final MechanismRoot2d innerBL;
    private final MechanismRoot2d innerTR;

    public static ElevatorVisualizer getVisualizerInstance(){
        return instance;
    } 

    public ElevatorVisualizer() {

        // Mechanism2d
        elevator2d = new Mechanism2d(6, 25);
        
        // Elevator Frame

        // bottom left node
        elevatorBL = elevator2d.getRoot("Elevator BL", 0, 0);

        elevatorBL.append(new MechanismLigament2d(
            "Left Tower",
            14, 
            90,
            10, 
            new Color8Bit(Color.kOrange)
        ));

        elevatorBL.append(new MechanismLigament2d(
            "Bottom Tower",
            6, 
            0,
            10, 
            new Color8Bit(Color.kOrange)
        ));

        // top right node
        elevatorTR = elevator2d.getRoot("Elevator TR", 6, 14);

        elevatorTR.append(new MechanismLigament2d(
            "Right Tower", 
            14,
            -90,
            10,
            new Color8Bit(Color.kOrange)
        ));

        elevatorTR.append(new MechanismLigament2d(
            "Top Side",
            6,
            180, 
            10, 
            new Color8Bit(Color.kOrange)));

        //outerFrame

        // bottom left node
        outerBL = elevator2d.getRoot("Outer BL", 1, 1);
        outerBL.append(new MechanismLigament2d(
            "Left Side",
            14,
            90, 
            10,
            new Color8Bit(Color.kYellow)));
    
        outerBL.append(new MechanismLigament2d(
            "Bottom Side",
            4,
            0,
            10,
            new Color8Bit(Color.kYellow)));
        
        // top right node
        outerTR = elevator2d.getRoot("Outer TR", 5, 15);
        outerTR.append(new MechanismLigament2d(
            "Top Side",
            4,
            180,
            10,
            new Color8Bit(Color.kYellow)));

        outerTR.append(new MechanismLigament2d(
            "Right Side",
            14,
            -90,
            10,
            new Color8Bit(Color.kYellow)));
        

        //innerFrame

        // bottom left node
        innerBL = elevator2d.getRoot("Inner BL", 2, 2);

        innerBL.append(new MechanismLigament2d(
            "Left Side",
            2,
            90, 
            10,
            new Color8Bit(Color.kPink)));

        innerBL.append(new MechanismLigament2d(
            "Bottom Side",
            2,
            0,
            10,
            new Color8Bit(Color.kPink)));

        // top right node
        innerTR = elevator2d.getRoot("Inner TR", 4, 4);

        innerTR.append(new MechanismLigament2d(
            "Top Side",
            2,
            180,
            10,
            new Color8Bit(Color.kPink)));
            
        innerTR.append(new MechanismLigament2d(
            "Right Side",
            2,
            -90, 
            10,
            new Color8Bit(Color.kPink)));


        SmartDashboard.putData("Visualizers/Elevator", elevator2d);
    }

    public void update() {
        // the middle will be at targetHeight
        ElevatorSimu simu = ((ElevatorSimu) ElevatorSimu.getInstance());
        System.out.println("distance: " + simu.getSim().getPositionMeters());

        innerBL.setPosition(2, simu.getHeight() + 2);
        innerTR.setPosition(4, simu.getHeight() + 4);

        outerBL.setPosition(1, 1 + simu.getHeight() * Settings.Elevator.SCALE_FACTOR);
        outerTR.setPosition(5, 15 + simu.getHeight() * Settings.Elevator.SCALE_FACTOR);
    }

}