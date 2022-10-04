package com.stuypulse.robot.subsystems;

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

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Settings.Intake.Simulation.*;
import static com.stuypulse.robot.constants.Settings.Intake.Deployment.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;

public class SimIntake extends IIntake {
    // SIMULATION HARDWARE
    private final LinearSystemSim<N2, N1, N1> intakeSim;

    private final MechanismLigament2d intakeArm;

    private final SmartNumber currentAngle;

    private final SmartNumber driverState;

    // CONTROL
    private final Controller controller;

    private final SmartNumber targetAngle;

    public SimIntake() {
        setSubsystem("Intake");

        // SIMULATION
        intakeSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(kV.get(), kA.get()));

        driverState = new SmartNumber("Intake/Driver Simulation State", 0);

        Mechanism2d intake = new Mechanism2d(3, 4);
        MechanismRoot2d intakeRoot = intake.getRoot("Intake Root", 2, 0);
        intakeArm = new MechanismLigament2d("Intake Arm", 1, 0);
        intakeRoot.append(intakeArm);
        addChild("Intake Mechanism2d", intake);

        // CONTROL
        controller = new PIDController(kP, kI, kD);
        targetAngle = new SmartNumber("Intake/Angle", 0);

        currentAngle = new SmartNumber("Intake/Angle", 0);
    }

    public void reset() {
        currentAngle.set(0);
    }

    @Override
    public void acquire() {
        driverState.set(1);
    }

    @Override
    public void deacquire() {
        driverState.set(-1);
    }

    @Override
    public void stop() {
        driverState.set(0);
    }

    @Override
    public void pointAtAngle(double angle) {
        targetAngle.set(angle);
    }

    @Override
    public double getAngle() {
        return currentAngle.get();
    }

    public double getTargetAngle() {
        return targetAngle.get();
    }

    private double getOutput() {
        double output = controller.update(getTargetAngle(), getAngle());

        if (getAngle() < RETRACT_ANGLE.get()) {
            currentAngle.set(RETRACT_ANGLE.get());
            output = Math.max(output, 0);
        }
        if (getAngle() > EXTEND_ANGLE.get()) {
            currentAngle.set(EXTEND_ANGLE.get());
            output = Math.min(0, output);
        }
        SmartDashboard.putNumber("Intake/Sim Output", output);
        return output;
    }

    @Override
    public void periodic() {
        intakeSim.setInput(getOutput() * RoboRioSim.getVInVoltage());

        intakeSim.update(0.02);
        currentAngle.set(intakeSim.getOutput(0));
        intakeArm.setAngle(getAngle());
        RoboRioSim.setVInCurrent(BatterySim.calculateDefaultBatteryLoadedVoltage(
                intakeSim.getCurrentDrawAmps()));
    }
}
