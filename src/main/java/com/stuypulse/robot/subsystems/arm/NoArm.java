package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.network.SmartNumber;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Motors.Arm.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoArm extends Arm {

    private SmartNumber targetAngle;
    private CANSparkMax motor;
    public RelativeEncoder encoder;
    private Controller controller;

    public NullArm() {
        // motor = new CANSparkMax(ARM_MOTOR_PORT, MotorType.kBrushless);
        // RelativeEncoder encoder = motor.getEncoder();
        // ARM_MOTOR_CONFIG.configure(motor);
    }

    @Override
    public void setVoltage(double voltage) {
        // motor.setVoltage(voltage);
    }

    public void runMotor() {
        // motor.set(ARM_MOTOR_RUN);
    }

    public void reverseMotor() {
        // motor.set(ARM_MOTOR_REVERSE);
    }

    public void stop() {
        // motor.set(0);
    }

    // set built in cansparkmax encoder up, get angle from encoder
    public double getAngle() {
        // return encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Motor ", motor.get());
    }

}
