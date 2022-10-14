package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.IClimber;

public class ClimberToBottom extends ClimberToHeight{

    public ClimberToBottom(IClimber climber){
        super(climber, Settings.Climber.MIN_HEIGHT);
    }
}
