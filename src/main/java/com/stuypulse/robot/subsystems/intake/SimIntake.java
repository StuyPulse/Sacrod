package com.stuypulse.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
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

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.IIntake;

import static com.stuypulse.robot.constants.Settings.Intake.Deployment.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;

public class SimIntake extends IIntake {
    // SIMULATION HARDWARE
    private final LinearSystemSim<N2, N1, N1> intakeSim;
    private final MechanismLigament2d intakeArm;
    private final SmartNumber driverState;

    // CONTROL
    private final Controller controller;
    private final SmartNumber targetAngle;

    public SimIntake() {
        setSubsystem("Intake");

        // SIMULATION
        intakeSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(kV.get(), kA.get()));

        driverState = new SmartNumber("Intake/Driver Simulation State", 0);

        Mechanism2d intake = new Mechanism2d(2, 2);
        MechanismRoot2d intakeRoot = intake.getRoot("Intake Root", 1, 1);
        intakeArm = new MechanismLigament2d("Intake Arm", 1, 2);
        intakeRoot.append(intakeArm);
        addChild("Intake Mechanism2d", intake);

        // CONTROL
        controller = new PIDController(kP, kI, kD);
        targetAngle = new SmartNumber("Intake/Target Angle", 0);
    }

    public void reset() {
        intakeSim.setState(VecBuilder.fill(0, 0));
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
    public void extend() {
        pointAtAngle(EXTEND_ANGLE.get());
    }

    @Override
    public void retract() {
        pointAtAngle(RETRACT_ANGLE.get());
    }

    public void pointAtAngle(double angle) {
        targetAngle.set(angle);
    }

    public double getAngleDegrees() {
        return intakeSim.getOutput(0) * 360.0;
    }

    public double getTargetAngle() {
        return targetAngle.get();
    }

    @Override
    public void periodic() {
        intakeSim.setInput(MathUtil.clamp(
            controller.update(getTargetAngle(), getAngleDegrees()),
            -12, +12));

        intakeSim.update(Settings.DT);

        if (getAngleDegrees() < RETRACT_ANGLE.get() || getAngleDegrees() > EXTEND_ANGLE.get())
            intakeSim.setState(VecBuilder.fill(
                MathUtil.clamp(getAngleDegrees(), RETRACT_ANGLE.get(), EXTEND_ANGLE.get()), 0));

        intakeArm.setAngle(getAngleDegrees() + 90);
        RoboRioSim.setVInCurrent(BatterySim.calculateDefaultBatteryLoadedVoltage(
                intakeSim.getCurrentDrawAmps()));
        
        SmartDashboard.putNumber("Intake/Angle", getAngleDegrees());
        SmartDashboard.putNumber("Intake/Voltage", controller.getOutput());
    }
}
