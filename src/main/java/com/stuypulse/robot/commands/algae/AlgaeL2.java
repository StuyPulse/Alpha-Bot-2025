package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeL2 extends AlgaeSetPivot {
    
    public AlgaeL2(){
        super(Settings.Algae.L2_ANGLE);
        algae.acquire();
    }

    public void initialize(){
        super.initialize();
    }
}
