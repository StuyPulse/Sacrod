package com.stuypulse.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static com.stuypulse.robot.constants.Motors.*;
import static com.stuypulse.robot.constants.Ports.Climber.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*- 
* @author Ivan Chen
* @author Carmin Vuong
* @author Jennifer Ye
* @author Jiayu Yan
* @author Niki Chen
*/

public class Climber extends SubsystemBase {

    private final WPI_TalonSRX motor;
    private double speed;

    public Climber() {
        
        motor = new WPI_TalonSRX(MOTOR);
        CLIMBER.configure(motor); 

        this.speed = 0.0;
    
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    private void setMotor(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getCurrentAmps() {
        return Math.abs(motor.getSupplyCurrent());
    }

    public double getMotorSpeed() {
        return motor.get();
    }

    @Override
    public void periodic() {
        setMotor(speed);

        SmartDashboard.putNumber("Climber/Current Amps", getCurrentAmps());
        SmartDashboard.putNumber("Climber/Motor Speed", getMotorSpeed());
    }
}
