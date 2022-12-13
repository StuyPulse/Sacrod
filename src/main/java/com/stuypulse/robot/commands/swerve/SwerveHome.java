package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveHome extends InstantCommand {
    public SwerveHome(Swerve swerve) {
        super(() -> swerve.reset(new Pose2d()), swerve);
    }
}
