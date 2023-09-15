package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Shooter.FeederFF;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMaxShooter extends ShooterImpl {

    private SmartNumber target;
    private double lastTarget;

    private CANSparkMax shooterA;
    private CANSparkMax shooterB;
    private SparkMaxPIDController shooterPID;
    private SimpleMotorFeedforward shooterFeedforward;

    private CANSparkMax feeder;
    private SparkMaxPIDController feederPID;
    private SimpleMotorFeedforward feederFeedforward;

    public SparkMaxShooter() {
        //
        target = new SmartNumber("Shooter/Target RPM", 0.0);
        lastTarget = 0.0;

        //
        shooterA = new CANSparkMax(Ports.Shooter.SHOOTER_MOTOR, MotorType.kBrushless);
        shooterB = new CANSparkMax(Ports.Shooter.SHOOTER_FOLLOWER, MotorType.kBrushless);

        shooterA.enableVoltageCompensation(12.0);
        shooterPID = shooterA.getPIDController();
        shooterPID.setP(Settings.Shooter.ShooterPID.kP.doubleValue());
        shooterPID.setI(Settings.Shooter.ShooterPID.kI.doubleValue());
        shooterPID.setD(Settings.Shooter.ShooterPID.kD.doubleValue());
        shooterPID.setFF(0.0);
        shooterPID.setDFilter(1.0);

        shooterFeedforward = new SimpleMotorFeedforward(
                Settings.Shooter.ShooterFF.kS.get(),
                Settings.Shooter.ShooterFF.kV.get(),
                Settings.Shooter.ShooterFF.kA.get());

        shooterB.follow(shooterA);

        Motors.Shooter.ShooterMotorConfig.configure(shooterA);
        Motors.Shooter.ShooterFollowerConfig.configure(shooterB);

        //
        feeder = new CANSparkMax(Ports.Shooter.FEEDER_MOTOR, MotorType.kBrushless);

        feeder.enableVoltageCompensation(12.0);
        feederPID = feeder.getPIDController();
        feederPID.setP(Settings.Shooter.FeederPID.kP.doubleValue());
        feederPID.setI(Settings.Shooter.FeederPID.kI.doubleValue());
        feederPID.setD(Settings.Shooter.FeederPID.kD.doubleValue());
        feederPID.setFF(0.0);
        feederPID.setDFilter(1.0);

        feederFeedforward = new SimpleMotorFeedforward(
                Settings.Shooter.FeederFF.kS.get(),
                Settings.Shooter.FeederFF.kV.get(),
                Settings.Shooter.FeederFF.kA.get());

        Motors.Shooter.FeederMotorConfig.configure(feeder);
    }

    public double getShooterTargetRPM() {
        return this.target.get();
    }

    public double getFeederTargetRPM() {
        return this.target.get() * FeederFF.FEEDER_RPM_MULTIPLIER.get();
    }

    @Override
    public void setTargetRPM(double targetRPM) {
        target.set(targetRPM);
    }

    @Override
    public double getShooterRPM() {
        return shooterA.getEncoder().getVelocity();
    }

    @Override
    public double getFeederRPM() {
        return feeder.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {

        shooterPID.setReference(
                lastTarget,
                ControlType.kVelocity,
                0,
                shooterFeedforward.calculate(lastTarget, target.get(), Settings.DT),
                ArbFFUnits.kVoltage);

        feederPID.setReference(
                lastTarget,
                ControlType.kVelocity,
                0,
                feederFeedforward.calculate(lastTarget, getFeederTargetRPM(), Settings.DT),
                ArbFFUnits.kVoltage);

        lastTarget = target.get();

        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
    }

}
