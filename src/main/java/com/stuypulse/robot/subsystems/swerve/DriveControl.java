package com.stuypulse.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.util.StopWatch;
import com.stuypulse.robot.constants.Settings.Swerve.DriveControl.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Methods:
 *  - setTargetVelocity(double)
 *  - getVeloctiy()
 * 
 * Fields:
 *  - Drive Motor
 *  - Encoder (CANSparkMax)
 *  - Controller
 *  - FeedForward
 *  - targetVelocity
 * 
 * @author Myles Pasetsky (myles@goldfisher.com)
 * @author Vincent Wang (vincent@goldfisher.com)
 * @author Benjamin Goldfisher (ben@goldfisher.com)
 * @author Shaurya Sen (shaurya@goldfisher.com)
 * @author Samuel Chen (sam@goldfisher.com)
 * @author Ivan Chen (ivan@goldfisher.com)
 */
public class DriveControl extends SubsystemBase {
	private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    
	private final Controller controller;
	private final SimpleMotorFeedforward feedforward;

    private double targetVelocity;
	private double prevTargetVelocity;
    private final StopWatch timer;

	public DriveControl(int deviceId) {
        motor = new CANSparkMax(deviceId, MotorType.kBrushless);
		encoder = motor.getEncoder();

		timer = new StopWatch();
		
        encoder.setPositionConversionFactor(Encoder.POSITION_CONVERSION);
		encoder.setVelocityConversionFactor(Encoder.VELOCITY_CONVERSION);

		targetVelocity = 0.0;
        prevTargetVelocity = 0.0;
    
        controller = Feedback.getFeedback();
        feedforward = FeedForward.getFeedForward();
	}

	public void setTargetVelocity(double velocity) { 
		targetVelocity = velocity;
	}

	public double getVelocity() {
		return encoder.getVelocity();
	}

	@Override
	public void periodic() {
		double targetAccel = (targetVelocity - prevTargetVelocity) / timer.reset();

		motor.set(feedforward.calculate(targetVelocity, targetAccel)
		        + controller.update(targetVelocity, getVelocity()));

        prevTargetVelocity = targetVelocity;
	}
}
