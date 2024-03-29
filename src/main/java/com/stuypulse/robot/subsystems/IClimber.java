package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.climber.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IClimber extends SubsystemBase {

    public static IClimber getInstance() {
        return RobotBase.isReal() ? new Climber() : new SimClimber();
    }

    public abstract void setTargetHeight(double height);

    public final void addTargetHeight(double delta) {
        setTargetHeight(getTargetHeight() + delta);
    }

    public abstract double getTargetHeight();

    public abstract double getHeight();

    public final boolean atHeight(double maxError) {
        return Math.abs(getHeight() - getTargetHeight()) < maxError;
    }

    public abstract void reset(double position);
    
}
