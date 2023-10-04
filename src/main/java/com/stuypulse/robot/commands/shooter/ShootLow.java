package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

public class ShootLow extends ShooterSetRPM {
    public ShootLow() {
        super(Settings.Scoring.LOW_RPM);
    }
}
