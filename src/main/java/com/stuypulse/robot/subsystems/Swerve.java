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
 * 
 * @author Myles Pasetsky (myles.pasetsky@gmail.com)
 * @author Vincent Wang (vincent@goldfisher.com)
 * @author Benjamin Goldfisher (ben@goldfisher.com)
 * @author Shaurya Sen (shauryasen12@gmail.com)
 * @author Samuel Chen (schen30@stuy.edu)
 * @author Ivan Chen (ivanchen07@gmail.com)
*/
public class Swerve extends SubsystemBase {
    
	private Module[] modules;

	public Swerve() {}
}
