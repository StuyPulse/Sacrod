package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.IShooter;

public class ShooterStop extends ShooterSetRPM {

    public ShooterStop(IShooter shooter) {
        super(shooter, 0);
    }
    
}
