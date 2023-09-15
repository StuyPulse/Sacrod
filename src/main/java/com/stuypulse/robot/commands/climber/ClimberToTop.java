package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

public class ClimberToTop extends ClimberToHeight {
    
    public ClimberToTop(Climber climber){
        super(climber, Settings.Climber.MAX_HEIGHT);
    }

}
