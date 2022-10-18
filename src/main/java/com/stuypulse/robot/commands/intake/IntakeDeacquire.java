package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.IIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeDeacquire extends CommandBase {

    private final IIntake intake;

    public IntakeDeacquire(IIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.deacquire();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
