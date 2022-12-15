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
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.THETA.kP, Motion.THETA.kI, Motion.THETA.kD),
			swerve::setStates,
			swerve
		);
	}

}
