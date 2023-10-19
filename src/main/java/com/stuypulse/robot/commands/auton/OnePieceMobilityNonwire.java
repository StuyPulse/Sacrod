package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.shooter.ShootFar;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceMobilityNonwire extends SequentialCommandGroup{
    public OnePieceMobilityNonwire(RobotContainer robot, String path) {
        // Time it takes for the shooter to reach the target speed
        double SHOOTER_INITIALIZE_DELAY = 0.3;

        PathPlannerTrajectory traj = PathPlanner.loadPath(path, Motion.CONSTRAINTS);
        
        addCommands(
				new ShootFar(),
				new WaitCommand(SHOOTER_INITIALIZE_DELAY),

				new FollowTrajectory(traj)
						.robotRelative());
    }
}
