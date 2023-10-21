package com.stuypulse.robot.subsystems.climber;

public class NoClimber extends Climber {

    @Override
    public void setTargetHeight(double height) {}

    @Override
    public double getTargetHeight() {
        return 0;
    }

    @Override
    public double getHeight() {
        return 0;
    }

    @Override
    public void reset(double position) {}
    
}
