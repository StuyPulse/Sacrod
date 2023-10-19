package com.stuypulse.robot.commands.auton;

import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Scoring;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceMobilityWire extends SequentialCommandGroup{
    public OnePieceMobilityWire(RobotContainer robot, String path) {
        // Time it takes for the shooter to reach the target speed
        double SHOOTER_INITIALIZE_DELAY = 0.3;

        PathPlannerTrajectory traj = PathPlanner.loadPath(path, Motion.CONSTRAINTS);
        
        addCommands(
				new ShootHigh(),
				new WaitCommand(SHOOTER_INITIALIZE_DELAY),

				new FollowTrajectory(traj)
						.robotRelative());
    }
}