package com.stuypulse.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
* @author Ivan Chen
* @author Carmin Vuong
* @author Jennifer Ye
* @author Jiayu Yan
* @author Niki Chen
*/

public class Climber extends SubsystemBase {

    private final WPI_TalonSRX motor;

    private final Controller controller;
    private final SmartNumber target;

    public Climber() {
        motor = new WPI_TalonSRX(MOTOR);
        CLIMBER.configure(motor);

        motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        motor.setSensorPhase(false);

        controller = new PIDController(kP, kI, kD);

        target = new SmartNumber("Climber/Target Height", MIN_HEIGHT);
        reset(MIN_HEIGHT);
    }

    public void setTargetHeight(double height) {
        target.set(MathUtil.clamp(height, MIN_HEIGHT, MAX_HEIGHT));
    }

    public double getHeight() {
        return motor.getSelectedSensorPosition() * Encoder.CONVERSION_FACTOR;
    }

    public double getCurrentAmps() {
        return motor.getSupplyCurrent();
    }

    public double getMotorSpeed() {
        return motor.get();
    }

    public void reset(double position) {
        motor.setSelectedSensorPosition(position / Encoder.CONVERSION_FACTOR);
    }

    @Override
    public void periodic() {
        motor.setVoltage(controller.update(target.get(), getHeight()));

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Controller Output", controller.getOutput());

        SmartDashboard.putNumber("Climber/Current Amps", getCurrentAmps());
    }
}
