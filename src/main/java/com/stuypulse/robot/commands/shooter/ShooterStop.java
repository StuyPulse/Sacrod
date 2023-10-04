package com.stuypulse.robot.commands.shooter;

import com.stuypulse.stuylib.network.SmartNumber;

public class ShooterStop extends ShooterSetRPM {

    public ShooterStop() {
        super(new SmartNumber("Shooter Stop", 0));
    }

}
