package com.stuypulse.robot.commands.algae;

import com.stuypulse.robot.constants.Settings;

public class AlgaeReefKnockoff extends AlgaeSetPivot {
    
    public AlgaeReefKnockoff(){
        super(Settings.Algae.REEF_KNOCKOFF_ANGLE);
    }

    public void initialize(){
        super.initialize();
    }
}
