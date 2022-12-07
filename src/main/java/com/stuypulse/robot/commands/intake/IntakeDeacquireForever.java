package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.IIntake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeDeacquireForever extends InstantCommand {
    private final IIntake intake;
    
    public IntakeDeacquireForever(IIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.deacquire();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
