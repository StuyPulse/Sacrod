package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveResetHeading extends InstantCommand {

    private final SwerveDrive swerve;
    private final Rotation2d facing;

    public SwerveDriveResetHeading() {
        this(new Rotation2d(0));
    }

    public SwerveDriveResetHeading(Rotation2d facing) {
        this.swerve = SwerveDrive.getInstance();
        this.facing = facing;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.reset(new Pose2d(swerve.getTranslation(), (facing)));
    }

}