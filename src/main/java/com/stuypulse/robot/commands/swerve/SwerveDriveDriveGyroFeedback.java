package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveDriveGyroFeedback extends CommandBase {
    private static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }
    
    private SwerveDrive swerve;

    private VStream speed;
    private IStream turn;

    private AngleController gyroFeedback;

    public SwerveDriveDriveGyroFeedback(SwerveDrive swerve, Gamepad driver) {
        this.swerve = swerve;

        speed = VStream.create(driver::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.DEADBAND),
                x -> x.clamp(1.0),
                x -> vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.MAX_TELEOP_SPEED.get()),
                new VLowPassFilter(Settings.Driver.Drive.RC)
                // new VRateLimit(Settings.Driver.MAX_ACCELERATION)
            );

        turn = IStream.create(driver::getRightX)
            .filtered(
                x -> SLMath.deadband(x, Settings.Driver.DEADBAND.get()),
                x -> SLMath.spow(x, Settings.Driver.Turn.POWER.get()),
                x -> x * Settings.Driver.MAX_TELEOP_TURNING.get(),
                new LowPassFilter(Settings.Driver.Turn.RC)
            );

        gyroFeedback = new AnglePIDController(Settings.Driver.GyroFeedback.kP, Settings.Driver.GyroFeedback.kI, Settings.Driver.GyroFeedback.kD);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        final double omega = turn.get();
        swerve.setStates(
            speed.get(), 
            omega + gyroFeedback.update(Angle.fromRadians(omega * Settings.DT), Angle.kZero)
        );
    }
}
