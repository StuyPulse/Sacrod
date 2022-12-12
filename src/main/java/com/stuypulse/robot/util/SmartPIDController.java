package com.stuypulse.robot.util;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDCalculator;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

public class SmartPIDController extends Controller {

    private final PIDController controller;
    private final PIDCalculator calculator;

    private final SmartBoolean tuningPID;
    private final SmartBoolean tuningPD;

    public SmartPIDController(String namespace) {
        this(
            new SmartNumber(String.join(namespace, "/kP"), 0.0),
            new SmartNumber(String.join(namespace, "/kI"), 0.0),
            new SmartNumber(String.join(namespace, "/kD"), 0.0),
            new SmartNumber(String.join(namespace, "/Bang Bang"), 1.0)
        );
    }

    public SmartPIDController(Number kp, Number ki, Number kd, Number kb) {
        controller = new PIDController(kp, ki, kd);
        calculator = new PIDCalculator(kp);

        
    }

    public PIDController setIntegratorFilter(Number range, Number limit) {
        mIFilter =
                new IFilterGroup(
                        x -> range.doubleValue() <= 0 || isDone(range.doubleValue()) ? x : 0,
                        x -> limit.doubleValue() <= 0 ? x : SLMath.clamp(x, limit.doubleValue()));
        return this;
    }

    public final Controller setSetpointFilter(IFilter... setpointFilter) {
        mSetpointFilter = IFilter.create(setpointFilter);
        return this;
    }

    public final Controller setMeasurementFilter(IFilter... measurementFilter) {
        mMeasurementFilter = IFilter.create(measurementFilter);
        return this;
    }

    public final Controller setOutputFilter(IFilter... outputFilter) {
        
    }


    @Override
    protected double calculate(double setpoint, double measurement) {
        return 0;
    }
    
}
