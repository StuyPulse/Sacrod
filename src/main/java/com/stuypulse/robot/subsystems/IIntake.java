package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.intake.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IIntake extends SubsystemBase {

    public static IIntake getInstance() {
        return RobotBase.isReal() ? new Intake() : new SimIntake();
    }

    // intaking

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stop();

    // deployment

    public abstract void retract();

    public abstract void extend();
}
