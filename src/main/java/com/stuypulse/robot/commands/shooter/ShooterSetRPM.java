package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
/**
 * initialize - runs when a command is started
 * execute - runs while a command is active
 * isFinished - return whether or not the command is finished
 * end - run when the command is finished
 */

/*
 * ways commands end:
 * 
 * "natural" (organic) - ends because isFinished returned true
 * interrupted - a command ends via interruption
 *    - binding a command to a button (e.g. whileHeld -> release that button, the command is interrupted)
 *    - subsystem requirements
 */

/*
3.0 // double --> Number
6.94 // double --> Number
1 // int --> Number
0 // int --> Number
-153 // int --> Number

Number
 - doubleValue()
 - intValue()
 - longValue()

SmartNumber - extends -> Number
 - doubleValue() --> reads from a network table entry

new ShooterSetRPM(shooter, 3.0);
new ShooterSetRPM(shooter, new SmartNumber("Shooter/Best RPM", 6940));
*/

/**
 * 
 */
public class ShooterSetRPM extends InstantCommand {
    private Shooter shooter;
    private SmartNumber targetRPM;

    public ShooterSetRPM(SmartNumber targetRPM) {
        shooter = Shooter.getInstance();
        this.targetRPM = targetRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetRPM(targetRPM);
    }
}
