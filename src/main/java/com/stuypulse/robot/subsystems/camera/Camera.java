package com.stuypulse.robot.subsystems.camera;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.subsystems.camera.*;
import com.stuypulse.robot.subsystems.camera.SimCamera;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Camera extends SubsystemBase {

    private static Camera instance = null;

    public static Camera getInstance() {
        if (instance == null) {
            instance = RobotBase.isReal() ? new PVCamera() : new SimCamera(SwerveDrive.getInstance());
        }
        return instance;
    }

    public abstract double getLatency();

    public abstract boolean hasTarget();

    public abstract double getDistance();

    public abstract Angle getHorizontalOffset();

    public abstract Pose2d getRobotPose();
}
