package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;


public class AlgaeProcessorScore extends AlgaeSetPivot {
    
    public AlgaeProcessorScore(){
        super(Settings.Algae.PROCESSOR_ANGLE);
        
        algae.deacquire();
    }

    public void initialize(){
        super.initialize();
    }

}
