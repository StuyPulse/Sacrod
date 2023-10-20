package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveX extends InstantCommand {
    private SwerveDrive swerve;

    public SwerveX() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setXMode();
    }
}
