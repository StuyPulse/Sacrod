package com.stuypulse.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.stuypulse.robot.constants.Settings.Swerve.TurnControl.*;


/**
 * Methods:
 *  - setTargetAngle(Rotation2d angle)
 *  - getAngle()
 * 
 * Fields:
 *  - Turn Motor (CANSparkMax)
 *  - Absolute Encoder (DutyCycleEncoder)
 *  - AngleController
 *  - targetAngle
 */
public class TurnControl extends SubsystemBase {

    private final CANSparkMax turnMotor;
    private final DutyCycleEncoder encoder;
    private final AngleController controller;

    private Rotation2d targetAngle;

    private SmartAngle offset; 

    public TurnControl(int deviceId, int channel, SmartAngle offset) {

        turnMotor = new CANSparkMax(deviceId, MotorType.kBrushless);
        encoder = new DutyCycleEncoder(channel);
        controller = Feedback.getFeedback();
        targetAngle = new Rotation2d();

        this.offset = offset;
    }

    public void setTargetAngle(Rotation2d target) {
        this.targetAngle = target;
    }

    public Rotation2d getAngle() {
		return new Rotation2d(getRawRadians()).minus(offset.getRotation2d());
    }

    public double getRawRadians() {
        return Encoder.getRadians(encoder.getAbsolutePosition());
    }

    @Override
    public void periodic() {
        turnMotor.set(controller.update(targetAngle, getAngle()));
    }
}
