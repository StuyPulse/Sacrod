package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeAcquireForever extends InstantCommand {

    private final Intake intake;
    
    public IntakeAcquireForever() {
        intake = Intake.getInstance();
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
