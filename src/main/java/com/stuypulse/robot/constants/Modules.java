package com.stuypulse.robot.constants;


import com.stuypulse.robot.subsystems.swerve.DriveControl;
import com.stuypulse.robot.subsystems.swerve.Module;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.geometry.Translation2d;

import static com.stuypulse.robot.constants.Ports.Swerve.*;
import static com.stuypulse.robot.constants.Motors.Swerve.*;

public interface Modules {
    Module[] MODULES = {
        new Module(
            "Swerve/Front Right", 
            new Translation2d(+Settings.Swerve.SIZE.getX() / 2, -Settings.Swerve.SIZE.getY() / 2), 
            new DriveControl(FrontRight.DRIVE_MOTOR, Drive.getConfig()), 
            new TurnControl(
                FrontRight.TURN_MOTOR, 
                FrontRight.ENCODER, 
                new SmartAngle("Swerve/Front Right/Angle Offset", Angle.fromDegrees(125.0)).useDegrees(),
                Turn.getConfig()
            )
        ),
        
        new Module(
            "Swerve/Front Left", 
            new Translation2d(+Settings.Swerve.SIZE.getX() / 2, +Settings.Swerve.SIZE.getY() / 2), 
            new DriveControl(FrontLeft.DRIVE_MOTOR, Drive.getConfig()),
            new TurnControl(
                FrontLeft.TURN_MOTOR, 
                FrontLeft.ENCODER, 
                new SmartAngle("Swerve/Front Left/Angle Offset", Angle.fromDegrees(-145.0)).useDegrees(),
                Turn.getConfig()
            )
        ),
        
        new Module(
            "Swerve/Back Left", 
            new Translation2d(-Settings.Swerve.SIZE.getX() / 2, +Settings.Swerve.SIZE.getY() / 2), 
            new DriveControl(BackLeft.DRIVE_MOTOR, Drive.getConfig()),
            new TurnControl(
                BackLeft.TURN_MOTOR,
                BackLeft.ENCODER, 
                new SmartAngle("Swerve/Back Left/Angle Offset", Angle.fromDegrees(-35.0)).useDegrees(),
                Turn.getConfig()
            )
        ),

        new Module(
            "Swerve/Back Right", 
            new Translation2d(-Settings.Swerve.SIZE.getX() / 2, -Settings.Swerve.SIZE.getY() / 2), 
            new DriveControl(BackRight.DRIVE_MOTOR, Drive.getConfig()),
            new TurnControl(
                BackRight.TURN_MOTOR, 
                BackRight.ENCODER, 
                new SmartAngle("Swerve/Back Right/Angle Offset", Angle.fromDegrees(145.0)).useDegrees(),
                Turn.getConfig()
            )
        ),
    };
}
