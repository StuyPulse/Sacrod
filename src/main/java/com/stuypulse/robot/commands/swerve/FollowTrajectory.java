package com.stuypulse.robot.commands.swerve;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTrajectory extends PPSwerveControllerCommand {

	private final PathPlannerTrajectory path;

	private boolean robotRelative;
	private boolean shouldStop;
	
	public FollowTrajectory(PathPlannerTrajectory path) {
		super(
			path,
			SwerveDrive.getInstance()::getPose,
			SwerveDrive.getInstance().getKinematics(),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.XY.kP, Motion.XY.kI, Motion.XY.kD),
			new PIDController(Motion.THETA.kP, Motion.THETA.kI, Motion.THETA.kD),
			SwerveDrive.getInstance()::setStates,
			true,
			SwerveDrive.getInstance()
		);

		robotRelative = false;
		shouldStop = false;

		this.path = path;
	}

	public FollowTrajectory robotRelative() {
		robotRelative = true;
		return this;
	}

	public FollowTrajectory fieldRelative() {
		robotRelative = false;
		return this;
	}

	public FollowTrajectory withStop() {
		shouldStop = true;
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
			PathPlannerState initialState = PathPlannerTrajectory.transformStateForAlliance(
				path.getInitialState(), DriverStation.getAlliance());

			SwerveDrive.getInstance().reset(new Pose2d(
				initialState.poseMeters.getTranslation(),
				initialState.holonomicRotation
			));
		}

		super.initialize();
	}

	@Override
	public void end(boolean interrupted) {
		if (shouldStop) {
			SwerveDrive.getInstance().stop();
		}
	}

}
