package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.IClimber;

public class ClimberToTop extends ClimberToHeight {
    
    public ClimberToTop(IClimber climber){
        super(climber, Settings.Climber.MAX_HEIGHT);
    }

}
