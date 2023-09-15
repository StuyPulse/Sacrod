package com.stuypulse.robot.commands.climber;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climber.Climber;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.util.StopWatch;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberDrive extends CommandBase {
    
    private IStream velocity;
    private final Climber climber;
    private StopWatch stopwatch;

    public ClimberDrive(Climber climber, Gamepad operator) {
        this.climber = climber;
        this.velocity = IStream.create(operator::getLeftY)
            .filtered(x -> x * Settings.Climber.MAX_VELOCITY.get());
        addRequirements(climber);

    }

    @Override
    public void execute(){
        climber.addTargetHeight(stopwatch.reset() * velocity.get());
    }
}
