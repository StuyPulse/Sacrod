package com.stuypulse.robot.commands.climber;

import static com.stuypulse.robot.constants.Settings.Climber.*;
import com.stuypulse.robot.subsystems.Climber;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command moves the climber up and down. 
 * 
 * Takes in a Climber and a Gamepad. The speed is from a 
 * filtered stream of values coming from the gamepad. 
 *
 * Sets the speed of the climber in execute. 
 *
 * @author Ivan Chen
 * @author Carmin Vuong
 * @author Jennifer Ye
 * @author Jiayu Yan
 * @author Niki Chen
 */
public class ClimberMove extends CommandBase {
    
    private Climber climber;
    private IStream speed;

    public ClimberMove(Climber climber, Gamepad operator) {
        
        this.climber = climber;

        this.speed = IStream.create(() -> operator.getLeftY())
                        .filtered(
                            x -> SLMath.deadband(x, DEADBAND.get()));

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setSpeed(speed.get());
    }

    @Override
    public void end(boolean isInterrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
