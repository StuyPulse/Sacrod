package com.stuypulse.robot.commands.swerve;

import com.stuypulse.stuylib.control.angle.AngleController;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.stuypulse.robot.subsystems.SwerveDrive;

public class GridAlign extends CommandBase {
    SwerveDrive swerve;
    //controller, target angle
    //controller goes to targetangle (180 or 0 degrees)
    private final AngleController controller; 
    
    public GridAlign() {
        swerve = SwerveDrive.getInstance();
        addRequirements(swerve);
    }

}
