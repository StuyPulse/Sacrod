package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Field;
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
		double txDegrees = limelight.getTargetXAngle();

		Angle txAngle = Angle.fromDegrees(txDegrees);
		
		return txAngle;
	}

	public Angle getVerticalOffset() {
		double tyDegrees = limelight.getTargetYAngle();
		
		Angle tyAngle = Angle.fromDegrees(tyDegrees);

		return tyAngle;
	}

	public double getDistance() {
		Angle ty = getVerticalOffset();

		double distance = Field.HUB_HEIGHT / ty.tan();

		return distance;
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
