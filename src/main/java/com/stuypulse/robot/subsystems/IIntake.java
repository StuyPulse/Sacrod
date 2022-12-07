package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IIntake extends SubsystemBase {
    // intaking

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stop();

    // deployment

    public abstract void retract();

    public abstract void extend();
}
