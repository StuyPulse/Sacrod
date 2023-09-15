package com.stuypulse.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Motors.*;
import static com.stuypulse.robot.constants.Ports.Climber.*;
import static com.stuypulse.robot.constants.Settings.Climber.*;
import static com.stuypulse.robot.constants.Settings.Climber.Feedback.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Ivan Chen
 * @author Carmin Vuong
 * @author Jennifer Ye
 * @author Jiayu Yan
 * @author Niki Chen
 */

public class ClimberImpl extends Climber {

    private final WPI_TalonSRX motor;

    private final Controller controller;
    private final SmartNumber target;

    public ClimberImpl() {
        motor = new WPI_TalonSRX(MOTOR);
        CLIMBER.configure(motor);

        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        motor.setSensorPhase(false);

        controller = new PIDController(kP, kI, kD);
        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(12);

        target = new SmartNumber("Climber/Target Height", MAX_HEIGHT);
        reset(MAX_HEIGHT);
    }

    @Override
    public void setTargetHeight(double height) {
        target.set(MathUtil.clamp(height, MIN_HEIGHT, MAX_HEIGHT));
    }

    @Override
    public double getTargetHeight() {
        return target.doubleValue();
    }

    @Override
    public double getHeight() {
        return motor.getSelectedSensorPosition() * Encoder.CONVERSION_FACTOR;
    }

    @Override
    public void reset(double position) {
        motor.setSelectedSensorPosition(position / Encoder.CONVERSION_FACTOR);
    }

    public double getCurrentAmps() {
        return motor.getSupplyCurrent();
    }

    @Override
    public void periodic() {
        setTargetHeight(getTargetHeight());
        motor.set(controller.update(target.get(), getHeight()));

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Controller Output", controller.getOutput());

        SmartDashboard.putNumber("Climber/Current Amps", getCurrentAmps());
    }
}
