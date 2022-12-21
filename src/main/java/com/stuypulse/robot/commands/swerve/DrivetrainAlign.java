package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings.Driver;
import com.stuypulse.robot.constants.Settings.Field;
import com.stuypulse.robot.constants.Settings.Scoring;
import com.stuypulse.robot.constants.Settings.Scoring.Auton;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.IFuser;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainAlign extends CommandBase {

    private final ICamera camera;
    private final SwerveDrive swerve;

    private final AngleController turnController;
    private final IFuser angleError;

    private final Controller distanceController;

    private final BStream ready;

    public DrivetrainAlign(ICamera camera, SwerveDrive swerve) {
        this.camera = camera;
        this.swerve = swerve;

        angleError = new IFuser(
                Auton.FUSION_FILTER,
                () -> camera.getHorizontalOffset().toDegrees(),
                () -> swerve.getAngle().getDegrees());

        turnController = new AnglePIDController(Auton.kP, Auton.kI, Auton.kD)
                .setOutputFilter(x -> -x); // reversed b/c camera on back of robot

        distanceController = new PIDController(Auton.DISTkP, Auton.DISTkI, Auton.DISTkD);

        ready = BStream.create(() -> turnController.isDoneDegrees(Scoring.ACCEPTABLE_TURN_ERROR.get()))
                .and(() -> distanceController.isDone(Scoring.ACCEPTABLE_DISTANCE_ERROR.get()))
                .filtered(new BDebounce.Rising(Scoring.READY_TIME));

        addRequirements(swerve, camera);
    }

    @Override
    public void initialize() {
        angleError.reset();
    }

    @Override
    public void execute() {
        var translation = new Vector2D(Field.HUB.minus(swerve.getPose().getTranslation()))
                .clamp(Driver.MAX_TELEOP_SPEED.get());

        var speeds = new ChassisSpeeds(
                translation.x,
                translation.y,
                turnController.update(Angle.kZero, camera.getHorizontalOffset()));

        swerve.setStates(speeds);
    }

    @Override
    public boolean isFinished() {
        return ready.get();
    }

}
