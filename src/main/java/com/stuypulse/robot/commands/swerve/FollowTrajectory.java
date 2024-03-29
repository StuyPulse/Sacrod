package com.stuypulse.robot.commands.swerve;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTrajectory extends PPSwerveControllerCommand {

	private boolean robotRelative;
	private PathPlannerTrajectory path;
	private SwerveDrive swerve;
	
	public FollowTrajectory(SwerveDrive swerve, PathPlannerTrajectory path) {
		super(
			path,
			swerve::getPose,
			swerve.getKinematics(),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.THETA.kP, Motion.THETA.kI, Motion.THETA.kD),
			swerve::setStates,
			swerve
		);

		robotRelative = false;
		this.path = path;
		this.swerve = swerve;
	}

	public FollowTrajectory robotRelative() {
		robotRelative = true;
		return this;
	}

	public FollowTrajectory fieldRelative() {
		robotRelative = false;
		return this;
	}

	public FollowPathWithEvents withEvents(Map<String, Command> events) {
		return new FollowPathWithEvents(
			this,
			path.getMarkers(),
			new HashMap<String, Command>(events)
		);
	}

	public FollowPathWithEvents withEvents(HashMap<String, Command> events) {
		return new FollowPathWithEvents(
			this,
			path.getMarkers(),
			events
		);
	}

	@Override
	public void initialize() {
		if (robotRelative) {
			swerve.reset(path.getInitialHolonomicPose());
		}

		super.initialize();
	}

}
