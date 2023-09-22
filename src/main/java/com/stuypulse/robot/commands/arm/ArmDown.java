package com.stuypulse.robot.commands.arm;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.stuylib.network.SmartNumber;
import static com.stuypulse.robot.constants.Settings.Arm.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmDown extends CommandBase{
    private final Arm arm;
    private SmartNumber targetAngle;
    
    public ArmDown() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }
    
    @Override
    public void initialize() {        
        arm.extend();        
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getAngle() - ARM_RETRACT_ANGLE.getAsDouble()) < 10; //change error 
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }
}
