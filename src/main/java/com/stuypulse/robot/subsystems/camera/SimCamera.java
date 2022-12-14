package com.stuypulse.robot.subsystems.camera;

import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.math.Angle;

public class SimCamera extends ICamera {

    private final SwerveDrive swerve;

    public SimCamera(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    @Override
    public double getDistance() {
        if (!hasTarget()) {
            return 0.0;
        }
        return 0;
    }

    private Angle getRawHorizontalOffset() {
        // swerve.getPose
        return null;
    }

    @Override
    public Angle getHorizontalOffset() {
        return null;
    }

    @Override
    public boolean hasTarget() {
        return false;
    }
    
}
