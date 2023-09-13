package com.stuypulse.robot.subsystems.arm;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.stuypulse.robot.constants.Ports.Arm.*;
import static com.stuypulse.robot.constants.Motors.Arm.*;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {

    private CANSparkMax motor;
    public RelativeEncoder encoder;
    private SmartNumber targetAngle;
    private Controller controller;

    public ArmImpl() {
        motor = new CANSparkMax(ARM_MOTOR_PORT, MotorType.kBrushless);
        RelativeEncoder encoder = motor.getEncoder();
        ARM_MOTOR_CONFIG.configure(motor);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void runMotor() {
        motor.set(ARM_MOTOR_RUN);
    }

    public void reverseMotor() {
        motor.set(ARM_MOTOR_REVERSE);
    }

    // set built in cansparkmax encoder up, get angle from encoder
    public double getAngle() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm/Motor ", motor.get());
    }

}
