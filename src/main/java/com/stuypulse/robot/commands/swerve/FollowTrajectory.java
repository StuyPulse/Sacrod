package com.stuypulse.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;

public class FollowTrajectory extends PPSwerveControllerCommand {
	
	public FollowTrajectory(SwerveDrive swerve, PathPlannerTrajectory traj) {
		super(
			traj,
			swerve::getPose,
			swerve.getKinematics(),
			new PIDController(Motion.X.kP, Motion.X.kI, Motion.X.kD),
			new PIDController(Motion.Y.kP, Motion.Y.kI, Motion.Y.kD),
			new PIDController(Motion.Theta.kP, Motion.Theta.kI, Motion.Theta.kD),
			swerve::setStates,
			swerve
		);
	}

}
