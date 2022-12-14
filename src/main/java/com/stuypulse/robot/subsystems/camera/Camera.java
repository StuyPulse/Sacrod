package com.stuypulse.robot.subsystems.camera;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* todo: make ICamera, SimCamera */
public class Camera extends ICamera {

	private Limelight limelight;
	
	public Camera() {
		limelight = Limelight.getInstance(); 

		for (int port : Settings.Limelight.PORTS) {
            edu.wpi.first.net.PortForwarder.add(port, "limelight.local", port);
        }
        CameraServer.startAutomaticCapture();
	}

	public Angle getHorizontalOffset() {
		if (!hasTarget()) {
            Settings.reportWarning("Unable To Find Target! [getHorizontal() was called]");
            return Angle.kZero;
        }
		double txDegrees = limelight.getTargetXAngle();

		Angle txAngle = Angle.fromDegrees(txDegrees);
		
		return txAngle;
	}

	public Angle getVerticalOffset() {
		if (!hasTarget()) {
            Settings.reportWarning("Unable To Find Target! [getVerticalOffset() was called]");
            return Angle.kZero;
        }
		double tyDegrees = limelight.getTargetYAngle();
		
		Angle tyAngle = Angle.fromDegrees(tyDegrees);

		return tyAngle;
	}

	public double getDistance() {
		Angle ty = getVerticalOffset().add(Settings.Limelight.CAMERA_PITCH);

		double distance = (Settings.Field.HUB_HEIGHT-Settings.Limelight.CAMERA_HEIGHT) / ty.tan();

		return distance + Settings.Field.HUB_TO_CENTER + Settings.Limelight.CAMERA_TO_CENTER;
	}

	
    public boolean hasTarget() {
        return limelight.getValidTarget();
    }

	@Override
	public void periodic() {
		if (!limelight.isConnected()) {
			Settings.reportWarning("[WARNING] Limelight is disconnected.");
		}

		SmartDashboard.putNumber("Camera/Distance", getDistance());
	}

}
