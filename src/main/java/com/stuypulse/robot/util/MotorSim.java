package com.stuypulse.robot.util;

import java.util.function.Function;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * A simulated motor intended to be a simple replacement for
 * real motors. Internally uses a LinearSystemSim constructed
 * based on the given motor type. Holds a reference to an EncoderSim
 * which can be accessed through .getEncoder().
 * 
 * @author Ben Goldfisher
 */
public class MotorSim implements MotorController, Sendable {

    public enum MotorType {
        CIM(DCMotor::getCIM),
        FALCON(DCMotor::getFalcon500),
        NEO(DCMotor::getNEO),
        NEO550(DCMotor::getNeo550),
        ROMI(DCMotor::getRomiBuiltIn);

        private Function<Integer, DCMotor> dcmotor;

        private MotorType(Function<Integer, DCMotor> dcmotor) {
            this.dcmotor = dcmotor;
        }

        public DCMotor getMotor(int num) {
            return this.dcmotor.apply(num);
        }
    };

    private final DCMotorSim sim;
    private final EncoderSim encoder;

    private double voltage;
    private boolean disabled;
    private boolean inverted;

    public MotorSim(MotorType motor, int motorCount, double gearing, double momentOfInertia) {
        sim = new DCMotorSim(motor.getMotor(motorCount), gearing, momentOfInertia);

        encoder = new EncoderSim(this);
    }

    @Override
    public void disable() {
        disabled = true;
    }

    /**
     * @return current set voltage of motor in range -12.0, 12.0
     */
    @Override
    public double get() {
        return voltage;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    /**
     * Set target voltage of motor
     * 
     * @param speed voltage in range -12.0, 12.0
     */
    @Override
    public void set(double speed) {
        if (disabled) {
            speed = 0;
        }

        sim.setInput(this.voltage = speed);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public void setVoltage(double outputVolts) {
        set(outputVolts / 12.0);
    }

    @Override
    public void stopMotor() {
        set(0);
    }

    public void update(double dtSeconds) {
        sim.update(dtSeconds);
        encoder.update(dtSeconds);
    }

    public double getCurrentDrawAmps() {
        return sim.getCurrentDrawAmps();
    }

    protected double getRadPerSecond() {
        return sim.getAngularVelocityRadPerSec();
    }

    public EncoderSim getEncoder() {
        return encoder;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("MotorSim");
        builder.addDoubleProperty("angular velocity", this::getRadPerSecond, null);
        builder.addDoubleProperty("set speed", this::get, this::set);
        builder.addBooleanProperty("inverted", this::getInverted, this::setInverted);
    }

}