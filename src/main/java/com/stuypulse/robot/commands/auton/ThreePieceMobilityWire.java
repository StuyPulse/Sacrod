package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.intake.IntakeRetract;
import com.stuypulse.robot.commands.shooter.ShootFar;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceMobilityWire extends SequentialCommandGroup {

    public ThreePieceMobilityWire() {

        double SHOOTER_INITIALIZE_DELAY = 0.5;
        double INTAKE_ACQUIRE_TIME = 0.5;

        PathPlannerTrajectory traj = PathPlanner.loadPath("OnePieceMobilityWire", new PathConstraints(4, 3));
        PathPlannerTrajectory secondpiece = PathPlanner.loadPath("ThreePieceMobilityWirePiece2", new PathConstraints(2.5, 2));
        PathPlannerTrajectory thirdpiece = PathPlanner.loadPath("ThreePieceMobilityWirePiece3", new PathConstraints(2.5, 2));
        
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

        // Drive to CS, shoot second piece
        addCommands(
            new FollowTrajectory(secondpiece).fieldRelative(),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY)
        );

        // Intake and shoot third piece
        addCommands(
            new FollowTrajectory(thirdpiece).fieldRelative(),
            new ConveyorSetMode(ConveyorMode.AUTONINDEXING).withTimeout(INTAKE_ACQUIRE_TIME),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY),
            new IntakeRetract()
        );
    }

}
