package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Ports.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.Deployment.*;
import static com.stuypulse.robot.constants.Motors.Intake.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An intake subsystem controlled by a drive and deployment motor.
 * Fields:
 * - driver motor
 * - deployment motor
 * - deployment encoder
 * - controller
 * - Target Angle
 *
 * Methods:
 * - acquire/deacquire
 * - extend/retract
 * - point at angle
 * - getAngle (from encoder)
 * (periodic)
 */

/**
 * @author Vincent Wang (vincentw921)
 * @author Marc Jiang (marcjiang7)
 * @author Ian Jiang (ijiang05)
 * @author Zixi Feng (zixifeng12)
 */

public class IntakeImpl extends Intake {

    private WPI_TalonSRX driverMotor;
    private CANSparkMax deploymentMotor;
    private RelativeEncoder deploymentEncoder;

    private Controller controller;
    private SmartNumber targetAngle;

    public IntakeImpl() {
        driverMotor = new WPI_TalonSRX(DRIVER_MOTOR);
        DriverConfig.configure(driverMotor);

        deploymentMotor = new CANSparkMax(DEPLOYMENT_MOTOR, MotorType.kBrushless);
        deploymentEncoder = deploymentMotor.getEncoder();
        deploymentEncoder.setPositionConversionFactor(POSITION_CONVERSION);
        DeploymentConfig.configure(deploymentMotor);

        controller = new PIDController(kP, kI, kD);

        targetAngle = new SmartNumber("Intake/Target Angle", 0.0);
        reset(RETRACT_ANGLE.get());
    }

    public void acquire() {
        driverMotor.set(ACQUIRE_SPEED.get());
    }

    public void deacquire() {
        driverMotor.set(DEACQUIRE_SPEED.get());
    }

    public void stop() {
        driverMotor.set(0);
    }

    public void reset(double position) {
        deploymentEncoder.setPosition(position);
        targetAngle.set(position);
    }

    public void extend() {
        pointAtAngle(EXTEND_ANGLE.get());
    }

    public void retract() {
        pointAtAngle(RETRACT_ANGLE.get());
    }

    public void pointAtAngle(double angle) {
        this.targetAngle.set(MathUtil.clamp(angle, RETRACT_ANGLE.get(), EXTEND_ANGLE.get()));
    }

    public double getAngle() {
        return deploymentEncoder.getPosition();// * 360.0 / 28.0;
    }

    @Override
    public void periodic() {
        pointAtAngle(targetAngle.get());
        deploymentMotor.set(controller.update(targetAngle.get(), getAngle()));

        SmartDashboard.putNumber("Intake/Deployment Speed", deploymentMotor.get());
        SmartDashboard.putNumber("Intake/Deployment Angle", getAngle());
        SmartDashboard.putNumber("Intake/Driver speed", driverMotor.get());
    }

}
