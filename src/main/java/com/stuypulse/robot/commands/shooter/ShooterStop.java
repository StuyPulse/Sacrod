package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;

public class ShooterStop extends ShooterSetRPM {

    public ShooterStop(Shooter shooter) {
        super(shooter, 0);
    }
    
}
