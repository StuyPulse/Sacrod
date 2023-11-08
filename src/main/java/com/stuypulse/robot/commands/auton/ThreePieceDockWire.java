package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.intake.IntakeRetract;
import com.stuypulse.robot.commands.shooter.ShootCS;
import com.stuypulse.robot.commands.shooter.ShootFar;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveBalance;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceDockWire extends SequentialCommandGroup {

    public ThreePieceDockWire() {
        
        double SHOOTER_INITIALIZE_DELAY = 0.5;
        double INTAKE_ACQUIRE_TIME = 0.5;

        PathPlannerTrajectory traj = PathPlanner.loadPath("OnePieceMobilityWire", new PathConstraints(4, 3));
        PathPlannerTrajectory secondpiece = PathPlanner.loadPath("ThreePieceMobilityWirePiece2", new PathConstraints(2.5, 2));
        PathPlannerTrajectory thirdpiece = PathPlanner.loadPath("ThreePieceMobilityWirePiece3", new PathConstraints(2.5, 2));
        PathPlannerTrajectory cstraj = PathPlanner.loadPath("ThreePieceDockWire", Motion.CONSTRAINTS);
        
        // Shoot first piece
        addCommands(
            new ShootFar(),
            new WaitCommand(SHOOTER_INITIALIZE_DELAY),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY)
        );
        
        // Intake second piece
        addCommands(
            new IntakeExtend(),
            new IntakeAcquireForever(),
            new FollowTrajectory(traj).robotRelative()
        );

        // Shoot second piece
        addCommands(
            new FollowTrajectory(secondpiece).fieldRelative(),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY)
        );

        // Intake third piece
        addCommands(
            new FollowTrajectory(thirdpiece).fieldRelative(),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY),
            new IntakeRetract()
        );

        // Engage and shoot
        addCommands(
            new FollowTrajectory(cstraj).fieldRelative(),
            new ParallelCommandGroup(
                new SwerveDriveBalance(),
                new WaitCommand(1.5)
                    .andThen(new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY*4))),
            new ShooterStop(),
            new IntakeStop()
        );
    }
    
}
