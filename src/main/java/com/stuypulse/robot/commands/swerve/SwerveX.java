package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveX extends CommandBase {
    private SwerveDrive swerve;

    public SwerveX() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setXMode();
    }
}
