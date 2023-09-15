package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {

    public static Shooter getInstance() {
        return RobotBase.isReal() ? new Shooter() : new SimShooter();
    }

    public abstract void setTargetRPM(double targetRPM);

    public abstract double getShooterTargetRPM();

    public abstract double getFeederTargetRPM();

    public abstract double getShooterRPM();

    public abstract double getFeederRPM();

    public final boolean isReady(double acceptableError) {
        return Math.abs(getShooterTargetRPM() - getShooterRPM()) < acceptableError;
    }

}