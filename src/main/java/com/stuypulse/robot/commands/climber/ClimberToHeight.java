package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberToHeight extends CommandBase{
    
    private final Climber climber;
    private final double height; 
    private boolean instant;

    public ClimberToHeight(Climber climber, double height){
        this.climber = climber;
        this.height = height;
        instant = true;
        addRequirements(climber);
    }
    
    public final ClimberToHeight untilReady(){
        instant = false;
        return this;
    }


    @Override   
    public void initialize(){
        climber.setTargetHeight(height);
    }

    @Override
    public boolean isFinished(){
        if(!instant){
            return climber.atHeight(Settings.Climber.ERROR.get());
        }
        return true;
    }

}
