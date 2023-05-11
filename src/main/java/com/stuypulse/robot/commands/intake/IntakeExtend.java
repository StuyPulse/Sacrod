package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Fields:
 * Intake
 * 
 * Methods:
 * initialize
 * end
 */

public class IntakeExtend extends InstantCommand {
    private final IIntake intake;

    public IntakeExtend(IIntake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.extend();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
