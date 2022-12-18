package com.stuypulse.robot.subsystems.camera;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Field;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.stuylib.math.Angle;

public class PVCamera extends ICamera {

    private final PhotonCamera camera;
    private PhotonPipelineResult result;

    public PVCamera(String cameraName) {
        camera = new PhotonCamera(cameraName);
        result = null;
    }

    public PVCamera() {
        this("photonvision");
    }

    private void forceUpdateResult() {
        result = camera.getLatestResult();
    }

    private PhotonPipelineResult getResult() {
        if (result == null) {
            forceUpdateResult();
        }
        return result;
    }

    @Override
    public boolean hasTarget() {
        return getResult().hasTargets();
    }

    @Override
    public double getDistance() {
        if (!getResult().hasTargets()) {
            Settings.reportWarning("[WARNING]: getDistance() called with no targets");
            return 0.0;
        }
        return PhotonUtils.calculateDistanceToTargetMeters(
            Limelight.CAMERA_HEIGHT,
            Field.HUB_HEIGHT,
            Limelight.CAMERA_PITCH.toRadians(),
            Math.toRadians(getResult().getBestTarget().getPitch())
        );
    }

    @Override
    public Angle getHorizontalOffset() {
        if (!getResult().hasTargets()) {
            Settings.reportWarning("[WARNING]: getHorizontalOffset() called with no targets");
            return Angle.kZero;
        }
        return Angle.fromDegrees(getResult().getBestTarget().getYaw());
    }

    @Override
    public void periodic() {
        // TODO: ping a network table inside camera to see if it's connected?

        forceUpdateResult();
    }
    
}
