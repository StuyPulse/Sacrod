package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.intake.IntakeRetract;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShootFar;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveBalance;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDockWire extends SequentialCommandGroup {

    public TwoPieceDockWire() {

        double SHOOTER_INITIALIZE_DELAY = 0.5;
        double INTAKE_ACQUIRE_TIME = 0.5;
        
        PathPlannerTrajectory traj = PathPlanner.loadPath("OnePieceMobilityWire", new PathConstraints(4, 3));
        PathPlannerTrajectory csTraj = PathPlanner.loadPath("TwoPieceDockWire", new PathConstraints(2.5, 2));
        
        // Shoot first piece
        addCommands(
            new ShootFar(),
            new WaitCommand(SHOOTER_INITIALIZE_DELAY),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY)
        );

        // Drive to up cs position, intake second piece
        addCommands(
            new IntakeExtend(),
            new IntakeAcquireForever(),
            new ShootHigh(),
            new FollowTrajectory(traj).robotRelative(),
            new ConveyorSetMode(ConveyorMode.AUTONINDEXING).withTimeout(INTAKE_ACQUIRE_TIME),
            new IntakeStop()
        );

        // Engage and shoot
        addCommands(
            new FollowTrajectory(csTraj).fieldRelative(),
            new ParallelCommandGroup(
                new SwerveDriveBalance(),
                new WaitCommand(1.5)
                    .andThen(new IntakeRetract())
                    .andThen(new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(3))),
            new ShooterStop()
        );
    }
    
}
