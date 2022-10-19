package com.stuypulse.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static com.stuypulse.robot.constants.Settings.Shooter.*;

import com.stuypulse.robot.subsystems.IShooter;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimShooter extends IShooter{

    // simulation
    private final FlywheelSim shooterSim;
    private final EncoderSim shooterEncoderSim;

    private final FlywheelSim shooterFollowerSim;
    private final EncoderSim shooterFollowerEncoderSim;
    
    private final FlywheelSim feederSim;
    private final EncoderSim feederEncoderSim;

    // encoders
    private Encoder shooterEncoder;
    private Encoder shooterFollowerEncoder;
    private Encoder feederEncoder;

    private SmartNumber targetRPM;
    
    private Controller shooterController;
    private Controller feederController;


    public SimShooter() {

        shooterSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(ShooterFF.kV.get(), ShooterFF.kA.get()), 
            DCMotor.getNEO(1), 
            shooterGearing
            );

        shooterFollowerSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(ShooterFF.kV.get(), ShooterFF.kA.get()), 
            DCMotor.getNEO(1), 
            shooterGearing
            );
        
        feederSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(FeederFF.kV.get(), FeederFF.kA.get()),
            DCMotor.getNEO(1),
            feederGearing
            );

        this.shooterEncoderSim = new EncoderSim(shooterEncoder);
        this.shooterFollowerEncoderSim = new EncoderSim(shooterFollowerEncoder);
        this.feederEncoderSim = new EncoderSim(feederEncoder);
        
        targetRPM = new SmartNumber("Shooter/TargetRPM", 0.0);

        shooterController = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD)
            .add(new Feedforward.Flywheel(ShooterFF.kS, ShooterFF.kV, ShooterFF.kA ).velocity());
        feederController = new PIDController(FeederPID.kP, FeederPID.kI, FeederPID.kD)
            .add(new Feedforward.Flywheel(FeederFF.kS, FeederFF.kV, FeederFF.kA).velocity());
    }

    public double getShooterRPM(){
        return (shooterEncoderSim.getRate() + shooterFollowerEncoderSim.getRate()) /2;
    }
    
    public double getFeederRPM(){
        return feederEncoderSim.getRate();
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

        shooterSim.setInputVoltage(shooterVoltage);
        shooterFollowerSim.setInputVoltage(shooterVoltage);

        feederSim.setInputVoltage(feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter Voltage", shooterVoltage);
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
    }
    
}
