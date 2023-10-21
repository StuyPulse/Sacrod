package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.stuylib.network.SmartNumber;
import static com.stuypulse.robot.constants.Settings.Arm.*;
import static com.stuypulse.robot.constants.Settings.Arm.Deployment.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

/*one motor
 * one absolute encoder
 * 
 * set angle
 * controller that gives voltage to motor
 * 
 * methods
 * go down
 * go up
 */
public abstract class Arm extends SubsystemBase {

    private SmartNumber targetAngle;
    private Controller controller;

    private static final Arm instance;

    static {
        instance = new NoArm();
    }

    public static final Arm getInstance() {
        return instance;
    }


    public Arm() {
        targetAngle = new SmartNumber("Arm/Target Angle", 0.0);
        controller = new PIDController(kP, kI, kD);
    }

    public void extend() {
        pointAtAngle(ARM_EXTEND_ANGLE.get());
        runMotor();
    }

    public void retract() {
        pointAtAngle(ARM_RETRACT_ANGLE.get());
        reverseMotor();
    }

    public abstract void runMotor();

    public abstract void reverseMotor();

    public abstract void stop();

    public void pointAtAngle(double angle) {
        this.targetAngle.set(MathUtil.clamp(angle, ARM_RETRACT_ANGLE.get(), ARM_EXTEND_ANGLE.get()));
    }

    public abstract double getAngle();

    public abstract void setVoltage(double voltage);

    @Override
    public void periodic() {
        pointAtAngle(targetAngle.get());
        setVoltage(controller.update(targetAngle.get(), getAngle()));

        SmartDashboard.putNumber("Arm/Deployment Angle", getAngle());
    }

}
