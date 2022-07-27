package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This command stops the climber.
 * 
 * Takes in a Climber.
 *
 * Tells the climber to stop in initialize.
 *
 * @author Ivan Chen
 * @author Carmin Vuong
 * @author Jennifer Ye
 * @author Jiayu Yan
 * @author Niki Chen
 */
public class ClimberStop extends InstantCommand {

    private Climber climber;
    
    public ClimberStop(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.stop();
    }
}
