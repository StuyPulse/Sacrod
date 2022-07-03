package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.swerve.DriveControl;
import com.stuypulse.robot.subsystems.swerve.TurnControl;
import com.stuypulse.robot.subsystems.swerve.Module;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Methods:
 *  - setSpeed(ChassisSpeed) 
 *  - setSpeed(Vector2D, double, boolean)
 *  - setSpeed(SwerveModuleState...)
 *  - getPose()
 *  - getAngle()
 * 
 * Fields:
 *  - 4 Modules
 *  - navX
 *  - Kinematics
 *  - Odometry
*/
public class Swerve extends SubsystemBase {
    
	private Module[] modules;

	public Swerve() {}
}
