package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.intake.IntakeRetract;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.shooter.ShootCS;
import com.stuypulse.robot.commands.shooter.ShootFar;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveBalance;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoPieceDockWire extends SequentialCommandGroup {
    /**
     * @param robot
     * @param path
     * @param cspath
     * @param upCSpath
     */
    public TwoPieceDockWire(RobotContainer robot, String path, String cspath) {
        double SHOOTER_INITIALIZE_DELAY = 0.5;
        double INTAKE_ACQUIRE_TIME = 0.5;

        PathPlannerTrajectory traj = PathPlanner.loadPath(path, new PathConstraints(4, 3));
        PathPlannerTrajectory csTraj = PathPlanner.loadPath(cspath, new PathConstraints(1, 1));
        // PathPlannerTrajectory upCSTraj = PathPlanner.loadPath(upCSpath, Motion.CONSTRAINTS);
        
        addCommands( //shoot first piece
                new SwerveDriveResetHeading(),
                new ShootFar(),
                new WaitCommand(SHOOTER_INITIALIZE_DELAY),
                new ConveyorSetMode(ConveyorMode.SHOOTING).withTimeout(SHOOTER_INITIALIZE_DELAY),
                new IntakeExtend(),
                new IntakeAcquireForever()
        );

        addCommands( //drive from current position to up cs position, intake cube on the way
                new FollowTrajectory(traj).robotRelative(),
                new ConveyorSetMode(ConveyorMode.AUTONINDEXING).withTimeout(INTAKE_ACQUIRE_TIME),
                new IntakeRetract()
        );
        addCommands( //engage and shoot
                new FollowTrajectory(csTraj).fieldRelative(),
                new ShootCS(),
                new ParallelCommandGroup(
                    new SwerveDriveBalance(),
                    new ConveyorSetMode(ConveyorMode.SHOOTING).withTimeout(SHOOTER_INITIALIZE_DELAY*4)),
                new ShooterStop(),
                new IntakeStop()
        );
    }
    
}
