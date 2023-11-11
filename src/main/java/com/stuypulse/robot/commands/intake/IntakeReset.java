package com.stuypulse.robot.commands.intake;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeReset extends InstantCommand {
    
    public IntakeReset() {
        super(() -> {
            Intake.getInstance().reset(Settings.Intake.RETRACT_ANGLE.get());
        });
    }

}
