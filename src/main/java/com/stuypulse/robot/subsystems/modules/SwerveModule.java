package com.stuypulse.robot.subsystems.modules;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SwerveModule extends SubsystemBase {
    public SwerveModule() {

    }

    public abstract String getId();
    public abstract Translation2d getLocation();
    public abstract void setTargetState(SwerveModuleState state);
    public abstract SwerveModuleState getState();
}
