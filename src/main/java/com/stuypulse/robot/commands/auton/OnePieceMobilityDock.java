package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveBalance;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceMobilityDock extends SequentialCommandGroup{
    public OnePieceMobilityDock(RobotContainer robot, String path) {

        // Time it takes for the shooter to reach the target speed
        double SHOOTER_INITIALIZE_DELAY = 0.5;

        PathPlannerTrajectory traj = PathPlanner.loadPath(path, new PathConstraints(3.5, 2));
        
        addCommands(
                new ShootHigh(),
                new WaitCommand(SHOOTER_INITIALIZE_DELAY),
                new ConveyorSetMode(ConveyorMode.SHOOTING).withTimeout(SHOOTER_INITIALIZE_DELAY),
                new ShooterStop(),

                new FollowTrajectory(traj)
                        .robotRelative(),
                new SwerveDriveBalance());
}
}
