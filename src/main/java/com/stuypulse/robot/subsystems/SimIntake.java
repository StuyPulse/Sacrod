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
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;

import static com.stuypulse.robot.constants.Settings.Intake.Simulation.*;
import static com.stuypulse.robot.constants.Settings.Intake.Deployment.*;
import static com.stuypulse.robot.constants.Settings.Intake.*;

public class SimIntake extends IIntake {

    private final LinearSystemSim<N2, N1, N1> intakeSim;

    private final MechanismLigament2d intakeArm;

    private final SmartAngle currentAngle;

    private final Controller controller;

    private final SmartAngle targetAngle;

    public SimIntake() {
        setSubsystem("Intake");
        intakeSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(kV.get(), kA.get()));
        Mechanism2d intake = new Mechanism2d(3, 4);
        MechanismRoot2d intakeRoot = intake.getRoot("Intake Root", 2, 0);
        intakeArm = new MechanismLigament2d("Intake Arm", 1, 0);
        intakeRoot.append(intakeArm);
        addChild("Intake Mechanism2d", intake);
        controller = new PIDController(kP, kI, kD);
        targetAngle = new SmartAngle("Intake/Angle", Angle.fromDegrees(0));

        currentAngle = new SmartAngle("Intake/Angle", Angle.fromDegrees(0));
    }

    @Override
    public void acquire() {

    }

    @Override
    public void deacquire() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void pointAtAngle(double angle) {
        targetAngle.set(Angle.fromDegrees(angle));
    }

    @Override
    public double getAngle() {
        return currentAngle.getAngle().toDegrees();
    }

    public double getTargetAngle() {
        return targetAngle.getAngle().toDegrees();
    }

    private double getOutput() {
        double output = controller.update(getTargetAngle(), getAngle());
        // Check to make sure doesn't go out of range
        if (getAngle() < RETRACT_ANGLE.get()) {
            currentAngle.set(Angle.fromDegrees(RETRACT_ANGLE.get()));
            output = Math.max(output, 0);
        }
        if (getAngle() > EXTEND_ANGLE.get()) {
            currentAngle.set(Angle.fromDegrees(EXTEND_ANGLE.get()));
            output = Math.min(0, output);
        }
        SmartDashboard.putNumber("Intake/Sim Output", output);
        return output;
    }

    @Override
    public void periodic() {
        intakeSim.setInput(getOutput() * RoboRioSim.getVInVoltage());

        intakeSim.update(0.01);
        currentAngle.set(Angle.fromDegrees(intakeSim.getOutput(0)));
        intakeArm.setAngle(getAngle());
        RoboRioSim.setVInCurrent(BatterySim.calculateDefaultBatteryLoadedVoltage(
                intakeSim.getCurrentDrawAmps()));
    }
}
