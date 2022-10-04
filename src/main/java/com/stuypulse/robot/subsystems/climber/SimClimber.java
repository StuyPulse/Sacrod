package com.stuypulse.robot.subsystems.climber;

import com.stuypulse.robot.subsystems.IClimber;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import static com.stuypulse.robot.constants.Settings.Climber.*;
import static com.stuypulse.robot.constants.Settings.Climber.Feedback.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimClimber extends IClimber {

    private final LinearSystemSim<N2, N1, N1> sim;

    private final Controller controller;
    private final SmartNumber target;

    private final MechanismLigament2d climber;

    public SimClimber() {
        sim = new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(SysId.kV, SysId.kA));

        controller = new PIDController(kP, kI, kD);

        target = new SmartNumber("Climber/Target Height", MIN_HEIGHT);
        reset(MIN_HEIGHT);

        climber = new Mechanism2d(50, 50).getRoot("Elevator Root", 10, 0).append(
            new MechanismLigament2d("Elevator", 0, 0));
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
        return sim.getOutput(1);
    }

    @Override
    public void reset(double position) {
        sim.setState(VecBuilder.fill(position, 0));
    }

    @Override
    public void periodic() {
        climber.setLength(getHeight());

        SmartDashboard.putNumber("Climber/Height", getHeight());
        SmartDashboard.putNumber("Climber/Controller Output", controller.getOutput());

        SmartDashboard.putNumber("Climber/Current Amps", sim.getCurrentDrawAmps());
    }

    @Override
    public void simulationPeriodic() {
        sim.setInput(controller.update(target.get(), getHeight()));
    }
}
