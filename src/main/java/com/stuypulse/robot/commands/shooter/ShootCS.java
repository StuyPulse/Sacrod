package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.shooter.Shooter;

public class ShootCS extends ShooterSetRPM {

    public ShootCS() {
        super(Settings.Scoring.CS_RPM);
    }

}
