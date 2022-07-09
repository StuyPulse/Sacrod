package com.stuypulse.robot.subsystems;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Modules;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.robot.subsystems.swerve.Module;

import static com.stuypulse.robot.constants.Settings.Swerve.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
 *  - SwerveDriveKinematics
 *  - SwerveDriveOdometry
 * 
 * @author Myles Pasetsky (myles.pasetsky@gmail.com)
 * @author Vincent Wang (vincent@goldfisher.com)
 * @author Benjamin Goldfisher (ben@goldfisher.com)
 * @author Shaurya Sen (shauryasen12@gmail.com)
 * @author Samuel Chen (schen30@stuy.edu)
 * @author Ivan Chen (ivanchen07@gmail.com)
*/
public class Swerve extends SubsystemBase {
    
	private final Module[] modules;
	private final AHRS gyro;

    private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;

	public Swerve() {
        modules = Modules.MODULES;		
        gyro = new AHRS(SPI.Port.kMXP);
		kinematics =  new SwerveDriveKinematics(
            Arrays.stream(modules)
                .map(x -> x.getLocation())
                .toArray(Translation2d[]::new)
        );
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle());
	}

    /** GYROSCOPE */

    public Rotation2d getGyroAngle() {
		return gyro.getRotation2d();
    }

    /** ODOMETRY */

    public Pose2d getPose() {
		return odometry.getPoseMeters();
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public void reset(Pose2d pose)  {
		odometry.resetPosition(pose, getGyroAngle());
    }

    private void update() {
        odometry.update(getGyroAngle(), getModuleStates());
    }

    /** MODULES */

    private SwerveModuleState[] getModuleStates() {
        // Arrays.stream(modules).map(x -> x.getState()).toArray(SwerveModuleState[]::new);
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        
        return states;
    }

    public void setSpeeds(Vector2D velocity, double angVel, boolean fieldRelative) {
        if (fieldRelative) {
            ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x, angVel, getAngle());
            setSpeeds(chassisSpeed);
        } else {
            ChassisSpeeds chassisSpeed = new ChassisSpeeds(velocity.y, -velocity.x, angVel);
		    setSpeeds(chassisSpeed);
        }
    }

	public void setSpeeds(ChassisSpeeds chassisSpeed) {
        setSpeeds(kinematics.toSwerveModuleStates(chassisSpeed));   
	}
	
	public void setSpeeds(SwerveModuleState[] states) {
        if (states.length != modules.length) {
			throw new IllegalArgumentException("Number of desired module states does not match the number of modules. ");
        }
		
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);

		for (int i = 0; i < states.length; i++) {
			modules[i].setTargetState(states[i]);
		}
	}
	
    @Override
    public void periodic() {
        update();
        
        SmartDashboard.putNumber("Swerve/Pose X", getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Pose Y", getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Pose Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Angle", getAngle().getDegrees());
    }

}
