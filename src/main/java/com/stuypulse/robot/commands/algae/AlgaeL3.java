package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeL3 extends AlgaeSetPivot {
    
    public AlgaeL3(){
        super(Settings.Algae.L3_ANGLE);
        algae.acquireOver();
    }

    public void initialize(){
        super.initialize();
    }
}
