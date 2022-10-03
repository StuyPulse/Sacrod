package com.stuypulse.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Wrapper for a simulated motor that provides an encoder API.
 * Constructed through a MotorSim.
 * 
 * Default distance units are in radians.
 * 
 * NOTE: motor speed is accessed through MotorSim
 * 
 * @author Ben Goldfisher
 */
public class EncoderSim implements Sendable {

    private double position;

    private double positionConversion;

    private boolean inverted;

    protected EncoderSim() {
        position = 0;
        positionConversion = 1;
    }

    protected void update(double dtSeconds, double vel) {
        position += dtSeconds * vel;
    }

    public double getDistance() {
        if (inverted)
            return -position * positionConversion;
        else
            return position * positionConversion;
    }

    public double getPositionConversion() {
        return positionConversion;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    // distance is by default in radians
    public void setPositionConversion(double conversion) {
        this.positionConversion = conversion;
    }

    public void reset(double position) {
        this.position = position * positionConversion;
    }

    public void reset() {
        reset(0);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Encoder");

        builder.addDoubleProperty("Distance", this::getDistance, null);
        builder.addDoubleProperty("Position Conversion", this::getPositionConversion, this::setPositionConversion);
    }

}