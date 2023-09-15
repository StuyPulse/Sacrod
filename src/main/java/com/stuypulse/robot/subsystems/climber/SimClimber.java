package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Settings.Climber.*;
import static com.stuypulse.robot.constants.Settings.Climber.Feedback.*;

import com.stuypulse.robot.constants.Settings;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimClimber extends Climber {

    private final LinearSystemSim<N2, N1, N1> sim;

    private final Controller controller;
    private final SmartNumber target;

    private final MechanismLigament2d climber2d;

    public SimClimber() {
        sim = new LinearSystemSim<>(
                LinearSystemId.identifyPositionSystem(Feedforward.kV, Feedforward.kA));

        controller = new PIDController(kP, kI, kD);

        target = new SmartNumber("Climber/Target Height", MIN_HEIGHT);
        reset(MIN_HEIGHT);

        Mechanism2d mechanism = new Mechanism2d(150, 300);
        climber2d = new MechanismLigament2d("Climber", 0, 90.0);
        mechanism.getRoot("Climber Root", 75, 0).append(climber2d);

        SmartDashboard.putData("Climber View", mechanism);
    }

    @Override
    public void setTargetHeight(double height) {
        target.set(MathUtil.clamp(height, MIN_HEIGHT, MAX_HEIGHT));
    }

    @Override
    public double getTargetHeight() {
        return target.doubleValue();
    }

    @Override
    public double getHeight() {
        return sim.getOutput(0);
    }

    @Override
    public void reset(double position) {
        sim.setState(VecBuilder.fill(position, 0));
    }

    @Override
    public void periodic() {
        sim.setInput(
                RoboRioSim.getVInVoltage() *
                        MathUtil.clamp(controller.update(target.get(), getHeight()), -1, +1));

        sim.update(Settings.DT);
        RoboRioSim.setVInCurrent(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        if (getHeight() < MIN_HEIGHT || getHeight() > MAX_HEIGHT) {
            reset(MathUtil.clamp(getHeight(), MIN_HEIGHT, MAX_HEIGHT));
        }

        climber2d.setLength(25 + 275 * (getHeight() - MIN_HEIGHT) / (MAX_HEIGHT - MIN_HEIGHT));

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Controller Output", controller.getOutput());

        SmartDashboard.putNumber("Climber/Current Amps", sim.getCurrentDrawAmps());
    }

}
