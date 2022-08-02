package com.stuypulse.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotSim;
import com.stuypulse.robot.constants.Settings.RobotSim.IntakeSim;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Ports.Intake.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;
import static com.stuypulse.robot.constants.Motors.Intake.*;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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
    /** ROBOT SIMULATION */
    Mechanism2d intakeMechanism;
    MechanismRoot2d intakeRoot;
    MechanismLigament2d intake;
    MechanismLigament2d intakeSetPoint;
    private LinearSystemSim<N2, N1, N1> deploySim;
    private double deployOutput;

    private WPI_TalonSRX driverMotor;
    private CANSparkMax deploymentMotor;

    private RelativeEncoder deploymentEncoder;

    private Controller controller;
    private SmartNumber targetAngle;

    public Intake() {
        driverMotor = new WPI_TalonSRX(DRIVER_MOTOR);
        deploymentMotor = new CANSparkMax(DEPLOYMENT_MOTOR, MotorType.kBrushless);
        deploymentEncoder = deploymentMotor.getEncoder();

        DriverConfig.configure(driverMotor);
        DeploymentConfig.configure(deploymentMotor);

        controller = Deployment.getController();

        deploymentEncoder.setPositionConversionFactor(POSITION_CONVERSION);

        targetAngle = new SmartNumber("Intake/Target Angle", 0.0);

        /** INTAKE SIMULATOR */
        intakeMechanism = new Mechanism2d(RobotSim.MECHANISM_DIM.x, RobotSim.MECHANISM_DIM.y);
        intakeRoot = intakeMechanism.getRoot("Root", IntakeSim.ROOT.x, IntakeSim.ROOT.y);
        intake = intakeRoot.append(
                new MechanismLigament2d("Intake", IntakeSim.INTAKE_LENGTH,
                        Settings.Intake.RETRACT_ANGLE.doubleValue()));
        deploySim = new LinearSystemSim<>(
                LinearSystemId.identifyPositionSystem(IntakeSim.kV.get(), IntakeSim.kA.get()));
        intakeSetPoint = intakeRoot.append(
                new MechanismLigament2d("Intake Set Point", IntakeSim.INTAKE_LENGTH,
                        targetAngle.get()));
        deployOutput = 0.0;
        SmartDashboard.putData("Intake", intakeMechanism);
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
        this.targetAngle.set(angle);
    }

    public double getAngle() {
        return deploymentEncoder.getPosition();
    }

    @Override
    public void periodic() {
        deployOutput = controller.update(targetAngle.get(), getAngle());
        if (getAngle() < RETRACT_ANGLE.get()) {
            deploymentEncoder.setPosition(RETRACT_ANGLE.get());
            deployOutput = Math.max(deployOutput, 0);
        }
        if (getAngle() > EXTEND_ANGLE.get()) {
            deploymentEncoder.setPosition(EXTEND_ANGLE.get());
            deployOutput = Math.min(deployOutput, 0);
        }
        deploymentMotor.set(deployOutput);
        SmartDashboard.putNumber("Intake/Deployment Speed", deploymentMotor.get());
        SmartDashboard.putNumber("Intake/Deployment Angle", getAngle());
        SmartDashboard.putNumber("Intake/Driver speed", driverMotor.get());
    }

    @Override
    public void simulationPeriodic() {
        deploySim.setInput(deployOutput * RoboRioSim.getVInVoltage());

        deploySim.update(RobotSim.dT);
        deploymentEncoder.setPosition(deploySim.getOutput(0));
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                deploySim.getCurrentDrawAmps()));
        intake.setAngle(getAngle());
        intakeSetPoint.setAngle(targetAngle.get());
    }
}
