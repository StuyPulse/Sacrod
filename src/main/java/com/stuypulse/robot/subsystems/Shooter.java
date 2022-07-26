package com.stuypulse.robot.subsystems;

import static com.stuypulse.robot.constants.Ports.Shooter.*;
import static com.stuypulse.robot.constants.Settings.Shooter.*;
import static com.stuypulse.robot.constants.Motors.Shooter.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Shooter subsystem controlled using Feedforward and PID. 
 * 
 * @author Eric lin (@cire694)
 * @author Vincent Lin (@Shroomicus)
 * @author Anthony Chen (@achen318)
 * @author Reya Miller (@reyamiller)
 */
public class Shooter extends SubsystemBase {

    private SmartNumber targetRPM;

    private CANSparkMax shooterMotor;
    private CANSparkMax shooterFollower;

    private RelativeEncoder shooterMotorEncoder;
    private RelativeEncoder shooterFollowerEncoder;

    private PIDController shooterPID;
    private SimpleMotorFeedforward shooterFF;

    private CANSparkMax feederMotor;
    private RelativeEncoder feederMotorEncoder;

    private PIDController feederPID;
    private SimpleMotorFeedforward feederFF;

    public Shooter() {
        targetRPM = new SmartNumber("Shooter/TargetRPM", 0.0);

        shooterMotor = new CANSparkMax(SHOOTER_MOTOR, MotorType.kBrushless);
        ShooterMotorConfig.configure(shooterMotor);
        
        shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
        ShooterFollowerConfig.configure(shooterFollower);

        shooterMotorEncoder = shooterMotor.getEncoder();
        shooterFollowerEncoder = shooterFollower.getEncoder();
        
        shooterPID = ShooterPID.PID();
        shooterFF = ShooterFF.FF();

        feederMotor = new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);
        FeederMotorConfig.configure(feederMotor);
        
        feederMotorEncoder = feederMotor.getEncoder();

        feederPID = FeederPID.PID();
        feederFF = FeederFF.FF();
    }

    public void setTargetRPM(double RPM){
        targetRPM.set(RPM);
    }

    public double getShooterRPM() {
        return (shooterMotorEncoder.getVelocity() + shooterFollowerEncoder.getVelocity()) / 2;
    }

    public double getFeederRPM() {
        return feederMotorEncoder.getVelocity();
    }

    private double getShooterVoltage() {
        return shooterFF.calculate(targetRPM.get()) + shooterPID.update(targetRPM.get(), getShooterRPM());
    }

    private double getFeederVoltage() {
        final double feederTargetRPM = targetRPM.get() * FeederFF.FEEDER_RPM_MULTIPLIER;

        return feederFF.calculate(feederTargetRPM) + feederPID.update(feederTargetRPM, getFeederRPM());
    }
    
    @Override
    public void periodic() {
        final double shooterVoltage = getShooterVoltage();
        final double feederVoltage = getFeederVoltage();

        shooterMotor.setVoltage(shooterVoltage);
        shooterFollower.setVoltage(shooterVoltage);

        feederMotor.setVoltage(feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter Voltage", shooterVoltage);
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
    }
}
