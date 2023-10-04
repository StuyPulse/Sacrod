package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;

public class ShootHigh extends ShooterSetRPM {
    public ShootHigh() {
        super(Settings.Scoring.HIGH_RPM);
    }

}
