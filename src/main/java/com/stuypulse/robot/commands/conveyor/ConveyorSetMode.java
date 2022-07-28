package com.stuypulse.robot.commands.conveyor;

import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ConveyorSetMode extends CommandBase{
    private final ConveyorMode mode;
    private final Conveyor conveyor;

    public ConveyorSetMode(Conveyor conveyor, ConveyorMode mode) {
        this.mode = mode;
        this.conveyor = conveyor;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setMode(mode);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setMode(ConveyorMode.DEFAULT);
    }
}

