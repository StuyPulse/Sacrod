package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.subsystems.Climber;
import com.stuypulse.stuylib.input.Gamepad;

/*-
 * @author Ivan Chen
 */

public class ClimberMoveUp extends ClimberMove {
    
    public ClimberMoveUp(Climber climber, Gamepad gamepad) {
        super(climber, gamepad, true);
    }

}
