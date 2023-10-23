package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
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

    public ThreePieceMobilityNonwire(String path, String secondpath, String thirdpath) {
        // Time it takes for the shooter to reach the target speed
        double INTAKE_DELAY = 0.5;
        double SHOOTER_INITIALIZE_DELAY = 0.3;

        PathPlannerTrajectory traj = PathPlanner.loadPath(path, Motion.CONSTRAINTS);
        PathPlannerTrajectory secondpiece = PathPlanner.loadPath(secondpath, Motion.CONSTRAINTS);
        PathPlannerTrajectory thirdpiece = PathPlanner.loadPath(thirdpath, Motion.CONSTRAINTS);
        
        addCommands(
            new SwerveDriveResetHeading(),
            new ShootCS(),
            new WaitCommand(SHOOTER_INITIALIZE_DELAY),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY),
            new IntakeExtend(),
            new IntakeAcquireForever(),

			new FollowTrajectory(traj).robotRelative(),
            new WaitCommand(INTAKE_DELAY),
            new FollowTrajectory(secondpiece).fieldRelative(),
            new ConveyorSetMode(ConveyorMode.FORWARD).withTimeout(SHOOTER_INITIALIZE_DELAY*4),
            new ShooterStop()
        );
    }
}
