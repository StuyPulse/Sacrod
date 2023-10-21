package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MobilityAuton extends SequentialCommandGroup {
    
    private static final PathConstraints CONSTRAINTS = new PathConstraints(1, 1);
    
    public MobilityAuton(String path) {
        addCommands(
            new FollowTrajectory(
                PathPlanner.loadPath(path, CONSTRAINTS)
            ).robotRelative()
        );
    }
}
