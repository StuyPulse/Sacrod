package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IShooter extends SubsystemBase {

    public double targetRPM;

    public void setTargetRPM(double targetRPM){
        this.targetRPM = targetRPM;
    };

    public abstract double getShooterRPM();

    public abstract double getFeederRPM();

    public abstract double getDesiredShooterVoltage();

    public abstract double getDesiredFeederVoltage();

}