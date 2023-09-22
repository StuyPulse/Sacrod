package com.stuypulse.robot.commands.arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.stuypulse.robot.subsystems.arm.Arm;
import static com.stuypulse.robot.constants.Settings.Arm.*;

public class ArmUp extends CommandBase{
    private final Arm arm;

    public ArmUp() {
        arm = Arm.getInstance();
        addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.retract();
    }

    @Override
    public void execute(){}

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getAngle() - ARM_RETRACT_ANGLE.getAsDouble()) < 10; //change error
    }

    @Override
    public void end(boolean interrupted){
        arm.stop();
    }

}
