package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.intake.IntakeRetract;
import com.stuypulse.robot.commands.shooter.ShootCS;
import com.stuypulse.robot.commands.shooter.ShootFar;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreePieceMobilityNonwire extends SequentialCommandGroup {

    public ThreePieceMobilityNonwire() {

        // Time it takes for the shooter to reach the target speed
        double INTAKE_ACQUIRE_TIME = 0.5;
        double SHOOTER_INITIALIZE_DELAY = 0.3;

        PathPlannerTrajectory traj = PathPlanner.loadPath("OnePieceMobilityNonwire", Motion.CONSTRAINTS);
        PathPlannerTrajectory drivetoCS = PathPlanner.loadPath("ThreePieceMobilityNonwirePiece2", Motion.CONSTRAINTS);
        PathPlannerTrajectory thirdpiece = PathPlanner.loadPath("ThreePieceMobilityNonwirePiece3", Motion.CONSTRAINTS);
        
        // Shoot first piece
        addCommands(
            new ShootFar(),
            new WaitCommand(SHOOTER_INITIALIZE_DELAY),
            new ConveyorSetMode(ConveyorMode.SHOOTING).withTimeout(SHOOTER_INITIALIZE_DELAY),
            new ShooterStop()
        );

        // Intake second piece
        addCommands(
            new IntakeExtend(),
            new IntakeAcquireForever(),

			new FollowTrajectory(traj).robotRelative()
        );
        
        // Drive to CS, shoot second piece
        addCommands(
            new SwerveDriveResetHeading(),
            new ShootCS(),
            new FollowTrajectory(drivetoCS).robotRelative(),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY)
        );

        // Intake and shoot third piece
        addCommands(
            new IntakeExtend(),
            new IntakeAcquireForever(),
            
            new FollowTrajectory(thirdpiece).fieldRelative(),
            new IntakeRetract(),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY*4),
            new ShooterStop()
        );
    }
}
