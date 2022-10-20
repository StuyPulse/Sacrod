package com.stuypulse.robot.subsystems.shooter;

import static com.stuypulse.robot.constants.Ports.Shooter.*;
import static com.stuypulse.robot.constants.Settings.Shooter.*;
import static com.stuypulse.robot.constants.Motors.Shooter.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.subsystems.IShooter;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends IShooter{
    
    private SmartNumber targetRPM;

    private CANSparkMax shooterMotor;
    private CANSparkMax shooterFollower;

    private RelativeEncoder shooterMotorEncoder;
    private RelativeEncoder shooterFollowerEncoder;

    private CANSparkMax feederMotor;
    private RelativeEncoder feederMotorEncoder;

    private Controller shooterController;
    private Controller feederController;

    
    public Shooter() {
        targetRPM = new SmartNumber("Shooter/TargetRPM", 0.0);

        shooterMotor = new CANSparkMax(SHOOTER_MOTOR, MotorType.kBrushless);
        ShooterMotorConfig.configure(shooterMotor);
        
        shooterFollower = new CANSparkMax(SHOOTER_FOLLOWER, MotorType.kBrushless);
        ShooterFollowerConfig.configure(shooterFollower);

        shooterMotorEncoder = shooterMotor.getEncoder();
        shooterFollowerEncoder = shooterFollower.getEncoder();

        shooterController = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD)
            .add(new Feedforward.Flywheel(ShooterFF.kS, ShooterFF.kV, ShooterFF.kA ).velocity());
        feederController = new PIDController(FeederPID.kP, FeederPID.kI, FeederPID.kD)
            .add(new Feedforward.Flywheel(FeederFF.kS, FeederFF.kV, FeederFF.kA).velocity());

        feederMotor = new CANSparkMax(FEEDER_MOTOR, MotorType.kBrushless);
        FeederMotorConfig.configure(feederMotor);
        
        feederMotorEncoder = feederMotor.getEncoder();

    }
    
    public void setTargetRPM(double targetRPM){
        this.targetRPM.set(targetRPM);
    }

    public double getShooterRPM(){
        return (shooterMotorEncoder.getVelocity() + shooterFollowerEncoder.getVelocity()) / 2;
    }
    
    public double getFeederRPM(){
        return feederMotorEncoder.getVelocity();
    }

    public double getDesiredShooterVoltage(){
        return shooterController.update(targetRPM.get(), getShooterRPM()); 
    }

    public double getDesiredFeederVoltage(){
        final double feederTargetRPM = targetRPM.get() * FeederFF.FEEDER_RPM_MULTIPLIER.get();
        return feederController.update(feederTargetRPM, getFeederRPM());
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
