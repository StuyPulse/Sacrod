package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.camera.LLCamera;
import com.stuypulse.robot.subsystems.camera.SimCamera;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ICamera extends SubsystemBase {

    public static ICamera getInstance(SwerveDrive swerve) {
        return RobotBase.isReal() ? new LLCamera(): new SimCamera(swerve);
    }

    public abstract boolean hasTarget();
    public abstract double getDistance();
    public abstract Angle getHorizontalOffset();

}
