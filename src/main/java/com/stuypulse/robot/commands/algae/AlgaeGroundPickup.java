package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeGroundPickup extends AlgaeSetPivot {
    
    public AlgaeGroundPickup(){
        super(Settings.Algae.GROUND_PICKUP_ANGLE);
        algae.acquireUnder();
    }
    public void initialize(){
        super.initialize();
    }
     
}