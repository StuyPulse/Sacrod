package com.stuypulse.robot.commands.swerve;


import static com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.constants.Settings.Swerve.Controls.*;
import com.stuypulse.robot.subsystems.Swerve;
import com.stuypulse.robot.util.VMath;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * Functionality:
 *  - left stick is translational velocity
 *     --> movement follows left stick
 *  - x-axis of right stick is angular velocity 
 *     --> angular velocity is x-value of right stick, 
 *         push to left turns counter-clockwise
 *         push to right turns clockwise
 *
 * Fields:
 *  - swerve drive
 *  - gamepad
 * 
 * Methods:
 *  - execute <-- feed in gamepad to swerve drive
 */
public class SwerveDrive extends CommandBase {
    private final Swerve swerve;
    private final Gamepad gamepad; 
    
    private final IFilter turnFilter;
    private final VFilter driveFilter;
    
    public SwerveDrive(Swerve swerve, Gamepad gamepad) {
        this.swerve = swerve;
        this.gamepad = gamepad;

        turnFilter = IFilter.create(
            x -> SLMath.deadband(x, Turn.DEADBAND.get()),
            x -> SLMath.spow(x, Turn.POWER.get()),
            new LowPassFilter(Turn.RC),
            x -> x * MAX_ANGULAR_SPEED
        );
        driveFilter = VFilter.create(
            new VDeadZone(Drive.DEADBAND),
            v -> VMath.spow(v, Drive.POWER.get()),
            new VLowPassFilter(Drive.RC),
            v -> v.mul(MAX_SPEED)
        );

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setSpeeds(
            driveFilter.get(gamepad.getLeftStick()),
            turnFilter.get(gamepad.getRightX()), 
            true
        );
    }
}