package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IShooter extends SubsystemBase {

    public static IShooter getInstance() {
        return RobotBase.isReal() ? new Shooter() : new SimShooter();
    }

    public abstract void setTargetRPM(double targetRPM);

    public abstract double getShooterRPM();

    public abstract double getFeederRPM();

}