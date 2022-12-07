package com.stuypulse.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.LinearSystemId;
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
    private final FlywheelSim feederSim;

    private SmartNumber targetRPM;
    
    private Controller shooterController;
    private Controller feederController;


    public SimShooter() {

        shooterSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(ShooterFF.kV.get(), ShooterFF.kA.get()), 
            DCMotor.getNEO(2), 
            shooterGearing
        );
        
        feederSim = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(FeederFF.kV.get(), FeederFF.kA.get()),
            DCMotor.getNEO(1),
            feederGearing
        );
        
        targetRPM = new SmartNumber("Shooter/Target RPM", 0.0);

        shooterController = new PIDController(ShooterPID.kP, ShooterPID.kI, ShooterPID.kD)
            .add(new Feedforward.Flywheel(ShooterFF.kS, ShooterFF.kV, ShooterFF.kA ).velocity());
        feederController = new PIDController(FeederPID.kP, FeederPID.kI, FeederPID.kD)
            .add(new Feedforward.Flywheel(FeederFF.kS, FeederFF.kV, FeederFF.kA).velocity());
    }

    public void setTargetRPM(double targetRPM){
        this.targetRPM.set(targetRPM);
    }
    public double getShooterRPM(){
        return shooterSim.getAngularVelocityRPM();
    }
    
    public double getFeederRPM(){
        return feederSim.getAngularVelocityRPM();
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
        feederSim.setInputVoltage(feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter Voltage", shooterVoltage);
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
    }
    
}
