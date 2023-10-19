package com.stuypulse.robot.subsystems.shooter;

import static com.stuypulse.robot.constants.Ports.Shooter.*;
import static com.stuypulse.robot.constants.Settings.Shooter.*;
import static com.stuypulse.robot.constants.Motors.Shooter.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {

    private SmartNumber targetRPM;

    private CANSparkMax shooterMotor;
    private CANSparkMax shooterFollower;

    private RelativeEncoder shooterMotorEncoder;
    private RelativeEncoder shooterFollowerEncoder;

    private CANSparkMax feederMotor;
    private RelativeEncoder feederMotorEncoder;

    private Controller shooterController;
    private Controller feederController;

    public ShooterImpl() {
        targetRPM = new SmartNumber("Shooter/Target RPM", 0.0);

        shooterMotor = new CANSparkMax(SHOOTER_MOTOR, MotorType.kBrushless);
        ShooterMotorConfig.configure(shooterMotor);

        shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
        ShooterFollowerConfig.configure(shooterFollower);

        shooterMotorEncoder = shooterMotor.getEncoder();
        shooterFollowerEncoder = shooterFollower.getEncoder();

        shooterController = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD)
                .add(new MotorFeedforward(ShooterFF.kS, ShooterFF.kV, ShooterFF.kA).velocity());
        feederController = new PIDController(FeederPID.kP, FeederPID.kI, FeederPID.kD)
                .add(new MotorFeedforward(FeederFF.kS, FeederFF.kV, FeederFF.kA).velocity());

        feederMotor = new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);
        FeederMotorConfig.configure(feederMotor);

        feederMotorEncoder = feederMotor.getEncoder();

    }

    public void setTargetRPM(SmartNumber targetRPM) {
        this.targetRPM.set(targetRPM);
    }

    public double getShooterTargetRPM() {
        return this.targetRPM.get();
    }

    public double getFeederTargetRPM() {
        return this.targetRPM.get() * FeederFF.FEEDER_RPM_MULTIPLIER.get();
    }

    public double getShooterRPM() {
        return (shooterMotorEncoder.getVelocity() + shooterFollowerEncoder.getVelocity()) / 2;
    }

    public double getFeederRPM() {
        return feederMotorEncoder.getVelocity();
    }

    public double getDesiredShooterVoltage() {
        return shooterController.update(targetRPM.get(), getShooterRPM());
    }

    public double getDesiredFeederVoltage() {
        return feederController.update(getFeederTargetRPM(), getFeederRPM());
    }

    public void stop() {
        targetRPM.set(0);
    }

    @Override
    public void periodic() {
        final double shooterVoltage = getDesiredShooterVoltage();
        final double feederVoltage = getDesiredFeederVoltage();

        shooterMotor.setVoltage(shooterVoltage);
        shooterFollower.setVoltage(shooterVoltage);

        feederMotor.setVoltage(feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter Voltage", shooterVoltage);
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
    }

}
