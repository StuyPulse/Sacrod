package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

public class ShootFar extends ShooterSetRPM {
    public ShootFar() {
        super(Settings.Scoring.FAR_RPM);
    }
}
