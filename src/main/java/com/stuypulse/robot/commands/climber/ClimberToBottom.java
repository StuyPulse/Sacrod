package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ClimberToBottom extends CommandBase{

    private final Climber climber;
    
    public ClimberToBottom(Climber climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setTargetHeight(Settings.Climber.MIN_HEIGHT);
    }

    @Override
    public boolean isFinished(){
        return climber.atHeight();
    }
}
