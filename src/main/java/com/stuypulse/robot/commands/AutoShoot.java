package com.stuypulse.robot.commands;

import java.util.ResourceBundle.Control;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Field;
import com.stuypulse.robot.constants.Settings.Scoring;
import com.stuypulse.robot.constants.Settings.Scoring.AutoShot;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.IShooter;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.util.ConveyorMode;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.vectors.VStream;
import com.stuypulse.stuylib.streams.vectors.filters.VDeadZone;
import com.stuypulse.stuylib.streams.vectors.filters.VLowPassFilter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * AutoShoot: 
 */
public class AutoShoot extends CommandBase {

    private final SwerveDrive swerve;
    private final IShooter shooter;
    private final Camera camera;
    private final Conveyor conveyor;
    private final VStream speed;

    private final AngleController turnController;

    private final BStream shooting;

    public AutoShoot(RobotContainer robot, Gamepad gamepad) {
        this.swerve = robot.swerve;
        this.shooter = robot.shooter;
        this.conveyor = robot.conveyor;
        this.camera = robot.camera;

        speed = VStream.create(gamepad::getLeftStick)
            .filtered(
                new VDeadZone(Settings.Driver.DEADBAND),
                x -> x.clamp(1.0),
                x -> Settings.vpow(x, Settings.Driver.Drive.POWER.get()),
                x -> x.mul(Settings.Driver.MAX_TELEOP_SPEED.get()),
                new VLowPassFilter(Settings.Driver.Drive.RC)
                // new VRateLimit(Settings.Driver.MAX_ACCELERATION)
            );
        
        turnController = new AnglePIDController(AutoShot.kP, AutoShot.kI, AutoShot.kD);

        shooting = BStream.create(() -> shooter.isReady(Scoring.ACCEPTABLE_RPM.get()))
            .and(() -> swerve.getVelocity().getNorm() < Scoring.ACCEPTABLE_VELOCITY.get())
            .and(() -> turnController.getError().toDegrees() < Scoring.ACCEPTABLE_TURN_ERROR.get())
            .filtered(new BDebounce.Rising(Scoring.READY_TIME));

        addRequirements(swerve, camera);
    }

    @Override
    public void initialize() {
    }

    private double getTargetShooterRPM() {
        return Settings.Scoring.DISTANCE_TO_RPM.interpolate(camera.getDistance());
    }

    private Angle getHorizontalOffset() {
        if (camera.hasAnyTarget()) {
            return camera.getHorizontalOffset();
        } else {
            var tx = Field.HUB.minus(swerve.getPose().getTranslation());
            return Angle.fromVector(tx.getX(), tx.getY());
        }
    }

    @Override
    public void execute() {
        shooter.setTargetRPM(getTargetShooterRPM());

        swerve.setStates(
            speed.get(), 
            turnController.update(Angle.kZero, getHorizontalOffset())
        );

        if (shooting.get()) {
            conveyor.setMode(ConveyorMode.SHOOTING);
        } else {
            conveyor.setMode(ConveyorMode.BRING_UP_BALLS);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
