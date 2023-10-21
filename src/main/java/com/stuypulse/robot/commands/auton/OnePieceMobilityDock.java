package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveBalance;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceMobilityDock extends SequentialCommandGroup {

    public OnePieceMobilityDock(String path, String csPath, String upCSPath) {

        // Time it takes for the shooter to reach the target speed
        double SHOOTER_INITIALIZE_DELAY = 0.5;

        // PathPlannerTrajectory traj = PathPlanner.loadPath(path, Motion.CONSTRAINTS);
        PathPlannerTrajectory traj = PathPlanner.loadPath(path, new PathConstraints(4, 3));
        PathPlannerTrajectory csTraj = PathPlanner.loadPath(csPath, new PathConstraints(1, 1));
        // PathPlannerTrajectory upCSTraj = PathPlanner.loadPath(upCSPath, Motion.CONSTRAINTS);
        PathPlannerTrajectory upCSTraj = PathPlanner.loadPath(upCSPath, new PathConstraints(4, 3));
        
        // shoot cube
        addCommands(
                new SwerveDriveResetHeading(),
                new ShootHigh(),
                new WaitCommand(SHOOTER_INITIALIZE_DELAY),
                new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY),
                new ShooterStop()
        );

        // go over cs, back, and balance
        addCommands(
                new FollowTrajectory(traj).robotRelative(),
                new FollowTrajectory(csTraj).fieldRelative(),
                new FollowTrajectory(upCSTraj).fieldRelative(),
                new SwerveDriveBalance()
        );
}
}
