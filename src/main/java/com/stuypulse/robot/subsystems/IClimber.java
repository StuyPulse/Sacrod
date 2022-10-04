package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IClimber extends SubsystemBase {

    public abstract void setTargetHeight(double height);

    public void addTargetHeight(double delta) {
        setTargetHeight(getTargetHeight() + delta);
    }

    public abstract double getTargetHeight();

    public abstract double getHeight();

    public boolean atHeight(double maxError) {
        return Math.abs(getHeight() - getTargetHeight()) < maxError;
    }

    public abstract void reset(double position);
    
}
