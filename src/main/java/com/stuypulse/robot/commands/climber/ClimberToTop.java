package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class ClimberToTop extends CommandBase{

    private final Climber climber;
    
    public ClimberToTop(Climber climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setTargetHeight(Settings.Climber.MAX_HEIGHT);
    }

    @Override
    public boolean isFinished(){
        return climber.atHeight();
    }
}
