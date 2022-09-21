package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ClimberMoveToHeight extends CommandBase{
    
    private final Climber climber;
    private final double height;

    public ClimberMoveToHeight(Climber climber, double height){
        this.climber = climber;
        this.height = height;
        addRequirements(climber);
    }

    @Override   
    public void initialize(){
        climber.setTargetHeight(height);
    }

    @Override
    public boolean isFinished(){
        return climber.atHeight();
    }


}
