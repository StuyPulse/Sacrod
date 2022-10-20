package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IShooter extends SubsystemBase {

    public abstract void setTargetRPM(double targetRPM);

    public abstract double getShooterRPM();

    public abstract double getFeederRPM();

}