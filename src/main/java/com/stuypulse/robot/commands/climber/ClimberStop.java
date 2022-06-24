package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/*-
 * @author Ivan Chen
 */

public class ClimberStop extends InstantCommand {
    
    private final Climber climber;

    public ClimberStop(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.stop();
    }

}
