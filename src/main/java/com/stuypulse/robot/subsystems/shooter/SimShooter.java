package com.stuypulse.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import static com.stuypulse.robot.constants.Settings.Shooter.*;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.IShooter;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimShooter extends IShooter{

    // simulation
    private final LinearSystemSim<N1, N1, N1> shooterSim;    
    private final LinearSystemSim<N1, N1, N1> feederSim;

    private SmartNumber targetRPM;
    
    private Controller shooterController;
    private Controller feederController;


    public SimShooter() {

        shooterSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(ShooterFF.kV.get(), ShooterFF.kA.get()));
        
        feederSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(FeederFF.kV.get(), FeederFF.kA.get()));
        
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
        return shooterSim.getOutput(0);
    }
    public double getShooterTargetRPM() {
        return this.targetRPM.get();
    }
    public double getFeederTargetRPM() {
        return this.targetRPM.get() * FeederFF.FEEDER_RPM_MULTIPLIER.get();
    }
    
    public double getFeederRPM(){
        return feederSim.getOutput(0);
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

        shooterSim.setInput(shooterVoltage);
        feederSim.setInput(feederVoltage);
        
        shooterSim.update(Settings.DT);
        feederSim.update(Settings.DT);

        SmartDashboard.putNumber("Shooter/Shooter Voltage", shooterVoltage);
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederVoltage);

        SmartDashboard.putNumber("Shooter/Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter/Feeder RPM", getFeederRPM());
    }
    
}
