package com.stuypulse.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.constants.Settings.Scoring;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class MylesAuto extends SequentialCommandGroup {

    // Time it takes for the shooter to reach the target speed
    private static final double SHOOTER_INITIALIZE_DELAY = 0.3;
    // Time it takes for the conveyor to give the shooter the ball
    private static final double CONVEYOR_TO_SHOOTER = 3.0;
    // Time we want to give the drivetrain to align
    private static final double DRIVETRAIN_ALIGN_TIME = 3.0;
	
	public MylesAuto(RobotContainer robot, String path) {
		HashMap<String, Command> events = new HashMap<>(); 
		events.put("align", new InstantCommand().withTimeout(DRIVETRAIN_ALIGN_TIME));
		events.put("shoot", new ConveyorSetMode(robot.conveyor, ConveyorMode.SHOOTING).withTimeout(CONVEYOR_TO_SHOOTER));

		SwerveAutoBuilder builder = new SwerveAutoBuilder(
			robot.swerve::getPose,
			robot.swerve::reset,
			Motion.XY,
			Motion.THETA,
			robot.swerve::setStates,
			events,
			robot.swerve
		);

		addCommands(
			// Init
			new IntakeExtend(robot.intake),
			new IntakeAcquireForever(robot.intake),
			new ShooterSetRPM(robot.shooter, Scoring.PRIMARY_RPM),
			new WaitCommand(SHOOTER_INITIALIZE_DELAY),

			builder.fullAuto(PathPlanner.loadPath(path, Motion.CONSTRAINTS))
		);
	}

}
