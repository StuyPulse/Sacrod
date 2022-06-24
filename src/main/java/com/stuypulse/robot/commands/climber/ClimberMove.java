package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Climber;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

/*-
 * @author Ivan Chen
 */

public class ClimberMove extends CommandBase {
    
    private final Climber climber;
    private final IStream speed;

    public ClimberMove(Climber climber, Gamepad gamepad) {
        this.climber = climber;
        this.speed = 
                IStream.create(() -> gamepad.getLeftY())
                    .filtered(
                        x -> SLMath.deadband(x, Settings.Climber.SPEED_DEADBAND.get()),
                        new LowPassFilter(Settings.Climber.SPEED_RC.get())
                    );
        
        addRequirements(climber);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        climber.setMotor(speed.get());
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
