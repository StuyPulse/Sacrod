package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MobilityAuton extends SequentialCommandGroup {
    public MobilityAuton(RobotContainer robot, String path) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(path, new PathConstraints(1, 1));
        
        addCommands(
            new FollowTrajectory(traj)
                .robotRelative()
        );
    }
}
