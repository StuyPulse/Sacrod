package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

public class ClimberToBottom extends ClimberToHeight{

    public ClimberToBottom(Climber climber){
        super(climber, Settings.Climber.MIN_HEIGHT);
    }
}
