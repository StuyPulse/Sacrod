package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MylesAuto extends SequentialCommandGroup {
	
	public MylesAuto(RobotContainer robot, String path) {
		PathPlannerTrajectory traj = PathPlanner.loadPath(path, Motion.CONSTRAINTS);

		addCommands(
			new InstantCommand(
				() -> { robot.swerve.reset(traj.getInitialHolonomicPose()); },
				robot.swerve
			),

			new FollowTrajectory(robot.swerve, traj)
		);
	}

}
