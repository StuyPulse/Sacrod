package com.stuypulse.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Methods:
 *  - setTargetState(SwerveModuleState)
 *  - getVelocity()
 *  - getAngle()
 *  - getId()
 *  - getLocation()
 * Fields:
 *  - DriveControl
 *  - TurnControl
 *  - id
 *  - targetState
 *  - location
 * 
 * @author Myles Pasetsky (myles@goldfisher.com)
 * @author Vincent Wang (vincent@goldfisher.com)
 * @author Benjamin Goldfisher (ben@goldfisher.com)
 * @author Shaurya Sen (shaurya@goldfisher.com)
 * @author Samuel Chen (sam@goldfisher.com)
 * @author Ivan Chen (ivan@goldfisher.com)
 */

public class Module extends SubsystemBase {
    private final String id;
    
    private final DriveControl driveControl;
    private final TurnControl turnControl;
    private final Translation2d location;

    private SwerveModuleState targetState;
    
    public Module(String id, Translation2d location, DriveControl driveControl, TurnControl turnControl) {

        this.targetState = new SwerveModuleState();
        this.id = "Swerve/" + id;
        this.driveControl = driveControl;
        this.turnControl = turnControl;

        this.location = location;
    }

    public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle());
    }

    public double getVelocity() {
        return driveControl.getVelocity();
    }

    public Rotation2d getAngle() {
        return turnControl.getAngle();
    }

    public String getId() {
        return this.id;
    }

    public Translation2d getLocation() {
        return location;
    }

    @Override
    public void periodic() {
        driveControl.setTargetVelocity(targetState.speedMetersPerSecond);
        turnControl.setTargetAngle(targetState.angle);
        
        SmartDashboard.putNumber(id + "/Target Velocity", targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(id + "/Target Angle", targetState.angle.getDegrees());   

        SmartDashboard.putNumber(id + "/Velocity", driveControl.getVelocity());
        SmartDashboard.putNumber(id + "/Angle", turnControl.getAngle().getDegrees());
    }
    
}