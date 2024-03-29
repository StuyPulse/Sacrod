package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;
import com.stuypulse.stuylib.streams.vectors.filters.VRateLimit;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveDrive extends CommandBase {
    
    private SwerveDrive swerve;

    private VStream speed;
    private IStream turn;

    public SwerveDriveDrive(SwerveDrive swerve, Gamepad driver) {
        this.swerve = swerve;

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.MAX_TELEOP_SPEED.get()),
                // new VLowPassFilter(Settings.Driver.Drive.RC)
                new VRateLimit(Settings.Driver.MAX_ACCELERATION)
            );

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Turn.POWER.get()),
                x -> x * Settings.Driver.MAX_TELEOP_TURNING.get(),
                new LowPassFilter(Settings.Driver.Turn.RC)
            );

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setStates(speed.get(), turn.get());
    }
}
