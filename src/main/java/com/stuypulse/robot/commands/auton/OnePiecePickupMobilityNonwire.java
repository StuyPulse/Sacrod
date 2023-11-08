package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.shooter.ShootFar;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePiecePickupMobilityNonwire extends SequentialCommandGroup {

    public OnePiecePickupMobilityNonwire() {

        // Time it takes for the shooter to reach the target speed
        double SHOOTER_INITIALIZE_DELAY = 0.3;

        PathPlannerTrajectory traj = PathPlanner.loadPath("OnePieceMobilityNonwire", Motion.CONSTRAINTS);
        
        // Shoot
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
    }
}
