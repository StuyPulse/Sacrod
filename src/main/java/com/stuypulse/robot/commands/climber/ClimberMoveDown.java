package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.Climber;
import com.stuypulse.stuylib.input.Gamepad;

/*-
 * @author Ivan Chen
 */

public class ClimberMoveDown extends ClimberMove {

    public ClimberMoveDown(Climber climber, Gamepad gamepad) {
        super(climber, gamepad, false);
    }
    
}
