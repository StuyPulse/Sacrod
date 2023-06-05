package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.intake.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IIntake extends SubsystemBase {

    private static IIntake instance = null;

    public static IIntake getInstance() {
        if (instance == null) {
            instance = RobotBase.isReal() ? new Intake() : new SimIntake();
        }
        return instance;
    }

    // intaking

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stop();

    // deployment

    public abstract void retract();

    public abstract void extend();
}
