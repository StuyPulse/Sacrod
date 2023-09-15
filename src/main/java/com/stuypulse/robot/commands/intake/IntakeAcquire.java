package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Commands we need:
 * 
 * Auton:
 * AcquireForever - instantaneous command
 * DeacquireForever - instantaneous command
 * 
 * Teleop:
 * Acquire while held
 * Deacquire while held
 * 
 * Intake retract
 * Intake extend
*/

/**
 * Methods: 
 * initialize
 * execute
 * isFinished
 * end
 * 
 * Fields:
 * intake
 */
public class IntakeAcquire extends CommandBase {
    private final Intake intake;
    
    public IntakeAcquire(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        intake.acquire();
    }
    
    @Override
    public void execute() {
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
