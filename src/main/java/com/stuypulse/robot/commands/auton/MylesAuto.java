package com.stuypulse.robot.commands.auton;

import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Scoring;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

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
		PathPlannerTrajectory traj = PathPlanner.loadPath(path, Motion.CONSTRAINTS);

		addCommands(
			// Init
			new IntakeExtend(),
			new IntakeAcquireForever(),
			new ShooterSetRPM(Scoring.PRIMARY_RPM),
			new WaitCommand(SHOOTER_INITIALIZE_DELAY),

			new FollowTrajectory(traj)
				.robotRelative()
				.withEvents(Map.of(
					"align", new InstantCommand().withTimeout(DRIVETRAIN_ALIGN_TIME),
					"shoot", new ConveyorSetMode(ConveyorMode.SHOOTING).withTimeout(CONVEYOR_TO_SHOOTER)
				))
		);
	}

}
