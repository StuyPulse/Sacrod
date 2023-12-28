package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

// we will be aiming with the swerve!
// use camera to get the distance to goal for RPM 
// for direction: 
/*
 * 1) get the differential; where will we be when we shoot the ball using a feed-forward model
 * 2) 
 * https://www.chiefdelphi.com/t/high-refresh-rate-vision-processing/387949/32
 * https://www.chiefdelphi.com/t/advice-on-shooting-while-moving/405472/2
 */
public class ShootWhileMoving extends CommandBase{
    
    private SwerveDrive swerve;
    private Pose2d target;
    private Rotation2d targetAngle;
    private Rotation2d currentAngle;
    private AnglePIDController angleController;



    public ShootWhileMoving(SwerveDrive swerve, Pose2d target){
        this.swerve = swerve;
        this.target = target;
        angleController = new AnglePIDController(1, 1, 1);


        addRequirements(swerve);
    }

    private void preaim(){
        targetAngle.plus(new Rotation2d(swerve.getVelocity().getX(), swerve.getVelocity().getY())); 
        // god i hope my logic is right!
        // the change in angle is just the change in x plus the change in y
    }

    @Override
    public void execute(){
        currentAngle = swerve.getAngle().plus(swerve.getGyroAngle());
        targetAngle = new Rotation2d(target.getX(), target.getY());
        preaim();

        
        swerve.setStates(new Vector2D(swerve.getVelocity()),
         angleController.update(Angle.fromRotation2d(targetAngle), Angle.fromRotation2d(currentAngle)));
    }
}
