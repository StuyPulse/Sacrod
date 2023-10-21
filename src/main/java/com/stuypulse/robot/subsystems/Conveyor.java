package com.stuypulse.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.ConveyorMode;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;

import static com.stuypulse.robot.constants.Ports.Conveyor.*;
import static com.stuypulse.robot.constants.Settings.Conveyor.*;
import static com.stuypulse.robot.constants.Motors.Conveyor.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
Fields:
-motor
-2 ir sensor

Methods:
- run conveyor
- run reverse
- stop

Logic:
- case indexing: 
   - if bottom ir has ball run until top ir true
   - if ir full do nothing
- case shooting: run until top ir empty
- case reverse: run in opposite direction
*/ 

/** 
 * @author Jason Zhou (independence106)
 * @author Andrew Liu (an6reww)
 * @author Tracey Lin (TracyLin)
 * @author Amber Shen (ambers7)
 * @author Yuchen Pan (Yuchen Pan) 
*/

public class Conveyor extends SubsystemBase {

    private final WPI_VictorSPX motor;

    private final DigitalInput intakeIR;
    private final DigitalInput shooterIR;

    private final BStream empty;
    private final BStream hasBall;

    private ConveyorMode mode;

    private static final Conveyor instance;

    static {
        instance = new Conveyor();
    }

    public static final Conveyor getInstance() {
        return instance;
    }

    public Conveyor() {
        motor = new WPI_VictorSPX(MOTOR); 
        MotorConfig.configure(motor);

        intakeIR = new DigitalInput(INTAKE_IR); 
        shooterIR = new DigitalInput(SHOOTER_IR);

        empty = BStream.create(this::IREmpty)
            .filtered(new BDebounceRC.Rising(EMPTY_DEBOUNCE))
            .polling(Settings.DT);

        hasBall = BStream.create(this::hasShooterBall)
            .or(this::hasIntakeBall)
            .filtered(new BDebounce.Falling(HASBALL_DEBOUNCE))
            .polling(Settings.DT);

        mode = ConveyorMode.DEFAULT;
    } 

    public void setMode(ConveyorMode conveyorMode) {
        mode = conveyorMode;
    }

    /** MOTOR INTERFACE **/

    public void runForward() {
        motor.set(FORWARD_SPEED.get());
    }

    public void runReverse() {
        motor.set(REVERSE_SPEED.get());
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getMotorSpeed() {
        return motor.get();
    }

    /** SENSORS **/

    public boolean hasShooterBall() { 
        return !shooterIR.get();
    }

    public boolean hasIntakeBall() {
        return !intakeIR.get();
    }

    private boolean IREmpty() {
        return !hasIntakeBall() && !hasShooterBall();
    }

    /** DEBOUNCES **/

    public boolean hasBall() {
        return hasBall.get();
    }
    
    public boolean isEmpty() {
        return empty.get();
    }

    @Override
    public void periodic() {
        mode.run(this);

        SmartDashboard.putBoolean("Conveyor/Has Shooter Ball", hasShooterBall());
        SmartDashboard.putBoolean("Conveyor/Has Intake Ball", hasIntakeBall());    
        SmartDashboard.putNumber("Conveyor/Motor Speed", getMotorSpeed());
        SmartDashboard.putString("Conveyor/Mode", mode.name());
    }

}
