package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquireForever extends InstantCommand {

    private final IIntake intake;
    
    public IntakeAcquireForever(IIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.acquire();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
