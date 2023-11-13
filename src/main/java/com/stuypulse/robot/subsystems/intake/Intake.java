package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.subsystems.arm.ArmImpl;
import com.stuypulse.robot.subsystems.intake.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    static {
        if (RobotBase.isReal()) {
            instance = new IntakeImpl();
        } else {
            instance = new SimIntake();
        }
    }

    public static final Intake getInstance() {
        return instance;
    }


    // intaking

    public abstract void acquire();

    public abstract void deacquire();

    public abstract void stop();

    // deployment

    public abstract void retract();

    public abstract void extend();

    public abstract void reset(double position);
}
