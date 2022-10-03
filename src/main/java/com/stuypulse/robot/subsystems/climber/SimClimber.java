package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.subsystems.IClimber;
import com.stuypulse.robot.util.EncoderSim;
import com.stuypulse.robot.util.MotorSim;
import com.stuypulse.robot.util.MotorSim.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Settings.Climber.*;
import static com.stuypulse.robot.constants.Settings.Climber.Feedback.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimClimber extends IClimber {

    private final MotorSim motor;
    private final EncoderSim encoder;

    private final Controller controller;
    private final SmartNumber target;

    private final MechanismLigament2d climber;

    public SimClimber() {
        motor = new MotorSim(MotorType.CIM, 1, 1.0, 1.0);
        encoder = motor.getEncoder();

        encoder.setPositionConversion(Encoder.CONVERSION_FACTOR);

        controller = new PIDController(kP, kI, kD);

        target = new SmartNumber("Climber/Target Height", MIN_HEIGHT);
        reset(MIN_HEIGHT);

        climber = new Mechanism2d(50, 50).getRoot("Elevator Root", 10, 0).append(
            new MechanismLigament2d("Elevator", 0, 0));
    }

    public void setTargetHeight(double height) {
        target.set(MathUtil.clamp(height, MIN_HEIGHT, MAX_HEIGHT));
    }

    public double getTargetHeight() {
        return target.doubleValue();
    }

    public void addTargetHeight(double delta) {
        setTargetHeight(getTargetHeight() + delta);
    }

    public double getHeight() {
        return encoder.getDistance();
    }

    public double getCurrentAmps() {
        return motor.getCurrentDrawAmps();
    }

    public double getMotorSpeed() {
        return encoder.getRadPerSecond();
    }

    public void reset(double position) {
        encoder.reset(position);
    }

    public boolean atHeight(){
        return controller.isDone(Settings.Climber.ERROR.get());
    }

    @Override
    public void periodic() {
        motor.setVoltage(controller.update(target.get(), getHeight()));

        climber.setLength(encoder.getDistance());

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Controller Output", controller.getOutput());

        SmartDashboard.putNumber("Climber/Current Amps", getCurrentAmps());
    }
}
