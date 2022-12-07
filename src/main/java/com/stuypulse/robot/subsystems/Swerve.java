package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    
    private final CANSparkMax[] motors;

    public Swerve() {
        motors = new CANSparkMax[] {
            new CANSparkMax(10, MotorType.kBrushless),
            new CANSparkMax(11, MotorType.kBrushless),
            new CANSparkMax(12, MotorType.kBrushless),
            new CANSparkMax(13, MotorType.kBrushless),
            new CANSparkMax(14, MotorType.kBrushless),
            new CANSparkMax(15, MotorType.kBrushless),
            new CANSparkMax(16, MotorType.kBrushless),
            new CANSparkMax(17, MotorType.kBrushless)
        };
    }

    public void periodic() {
        for (CANSparkMax motor : motors) {
            motor.set(0.0);
        }
    }

}
