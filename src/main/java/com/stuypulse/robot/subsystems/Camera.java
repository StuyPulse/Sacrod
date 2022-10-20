package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {

	private Limelight limelight;
	
	public Camera() {
		limelight = Limelight.getInstance(); 
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

		double distance = HUB_HEIGHT / ty.tan();

		return distance;
	}
}
