package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IClimber extends SubsystemBase {

    public abstract void setTargetHeight(double height);

    public abstract double getTargetHeight();

    public abstract void addTargetHeight(double delta);

    public abstract double getHeight();

    public abstract boolean atHeight();

    public abstract double getCurrentAmps();

    public abstract void reset(double position);
    
}
