package com.stuypulse.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Ports.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Motors.Intake.*;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

public class Intake extends SubsystemBase {

    private WPI_TalonSRX driverMotor;
    private CANSparkMax deploymentMotor;

    private Encoder deploymentEncoder;
    private EncoderSim encoderSim;

    private Controller controller;
    private SmartNumber targetAngle;

    public Intake() {
        driverMotor = new WPI_TalonSRX(DRIVER_MOTOR);
        deploymentMotor = new CANSparkMax(DEPLOYMENT_MOTOR, MotorType.kBrushless);
        //deploymentEncoder = deploymentMotor.getEncoder();
        deploymentEncoder = new Encoder(2, 3, false, EncodingType.k1X);

        DriverConfig.configure(driverMotor);
        DeploymentConfig.configure(deploymentMotor);

        controller = Deployment.getController();

        deploymentEncoder.setDistancePerPulse(POSITION_CONVERSION);

        targetAngle = new SmartNumber("Intake/Target Angle", 0.0);

        encoderSim = new EncoderSim(deploymentEncoder);

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

    private void reset(double position) {
        encoderSim.setDistance(position);
        targetAngle.set(position);
    }

    public void extend() {
        pointAtAngle(EXTEND_ANGLE.get());
    }

    public void retract() {
        pointAtAngle(RETRACT_ANGLE.get());
    }

    public void pointAtAngle(double angle) {
        this.targetAngle.set(angle);
    }

    public double getAngle() {
        return encoderSim.getDistance();
    }

    @Override
    public void periodic() {
        if (!controller.isDone(Deployment.MAX_ERROR.get())) {
            deploymentMotor.set(controller.update(targetAngle.get(),
                    getAngle()));
        } else {
            deploymentMotor.set(0);
        }

        SmartDashboard.putNumber("Intake/Deployment Speed", deploymentMotor.get());
        SmartDashboard.putNumber("Intake/Deployment Angle", getAngle());
        SmartDashboard.putNumber("Intake/Driver speed", driverMotor.get());
    }

}
