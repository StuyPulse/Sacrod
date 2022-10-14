package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeRetract extends InstantCommand {
    private final IIntake intake;
    
    public IntakeRetract(IIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }    

    @Override
    public void initialize() {
        intake.retract();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
