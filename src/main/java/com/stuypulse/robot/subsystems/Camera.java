package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* todo: make ICamera, SimCamera */
public class Camera extends SubsystemBase {

	private Limelight limelight;
	
	public Camera() {
		limelight = Limelight.getInstance(); 

		for (int port : Settings.Limelight.PORTS) {
            PortForwarder.add(port, "limelight.local", port);
        }
        CameraServer.startAutomaticCapture();
	}

	public Angle getHorizontalOffset() {
		if (!hasAnyTarget()) {
            Settings.reportWarning("Unable To Find Target! [getHorizontal() was called]");
            return Angle.kZero;
        }
		double txDegrees = limelight.getTargetXAngle();

		Angle txAngle = Angle.fromDegrees(txDegrees);
		
		return txAngle;
	}

	public Angle getVerticalOffset() {
		if (!hasAnyTarget()) {
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

	
    public boolean hasAnyTarget() {
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
