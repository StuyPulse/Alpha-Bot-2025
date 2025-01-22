package com.stuypulse.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import com.stuypulse.robot.constants.Settings;

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
        // Bottom Left Node 
        elevatorBL = elevator2d.getRoot("Elevator BL", 0, 0);

        elevatorBL.append(new MechanismLigament2d(
            "Left Tower",
            14, 
            90,
            10, 
            new Color8Bit(Color.kOrange)
            )
        );

        elevatorBL.append(new MechanismLigament2d(
            "Bottom Tower",
            6, 
            0,
            10, 
            new Color8Bit(Color.kOrange)
            )
        );

        // Top Right Node
        elevatorTR = elevator2d.getRoot("Elevator TR", 6, 14);

        elevatorTR.append(new MechanismLigament2d(
            "Right Tower", 
            14,
            -90,
            10,
            new Color8Bit(Color.kOrange)
            )
        );

        elevatorTR.append(new MechanismLigament2d(
            "Top Side",
            6,
            180, 
            10, 
            new Color8Bit(Color.kOrange)
            )
        );

        //Outer Frame
        // Bottom Left Node
        outerBL = elevator2d.getRoot("Outer BL", 1, 1);

        outerBL.append(new MechanismLigament2d(
            "Left Side",
            14,
            90, 
            10,
            new Color8Bit(Color.kYellow)
            )
        );
    
        outerBL.append(new MechanismLigament2d(
            "Bottom Side",
            4,
            0,
            10,
            new Color8Bit(Color.kYellow)
            )
        );
        
        // Top Right Node
        outerTR = elevator2d.getRoot("Outer TR", 5, 15);

        outerTR.append(new MechanismLigament2d(
            "Top Side",
            4,
            180,
            10,
            new Color8Bit(Color.kYellow)
            )
        );

        outerTR.append(new MechanismLigament2d(
            "Right Side",
            14,
            -90,
            10,
            new Color8Bit(Color.kYellow)
            )
        );
        
        // Inner Frame
        // Bottom Left Node
        innerBL = elevator2d.getRoot("Inner BL", 2, 2);

        innerBL.append(new MechanismLigament2d(
            "Left Side",
            2,
            90, 
            10,
            new Color8Bit(Color.kPink)
            )
        );

        innerBL.append(new MechanismLigament2d(
            "Bottom Side",
            2,
            0,
            10,
            new Color8Bit(Color.kPink)
            )
        );

        // Top Right Node
        innerTR = elevator2d.getRoot("Inner TR", 4, 4);

        innerTR.append(new MechanismLigament2d(
            "Top Side",
            2,
            180,
            10,
            new Color8Bit(Color.kPink)
            )
        );
            
        innerTR.append(new MechanismLigament2d(
            "Right Side",
            2,
            -90, 
            10,
            new Color8Bit(Color.kPink)
            )
        );

        SmartDashboard.putData("Visualizers/Elevator", elevator2d);
    }

    public void update() {
        // Middle of Inner Frame will be at Target Height
        ElevatorSimu simu = ((ElevatorSimu) ElevatorSimu.getInstance());
        System.out.println("Distance: " + simu.getSim().getPositionMeters());

        innerBL.setPosition(2, simu.getHeight() + 2);
        innerTR.setPosition(4, simu.getHeight() + 4);

        outerBL.setPosition(1, 1 + simu.getHeight() * Settings.Elevator.SCALE_FACTOR);
        outerTR.setPosition(5, 15 + simu.getHeight() * Settings.Elevator.SCALE_FACTOR);
    }

}