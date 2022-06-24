package com.stuypulse.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Climber.Stalling;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * @author Ivan Chen
 */

public class Climber extends SubsystemBase {
    
    private final WPI_TalonSRX climber;

    private final IStream speed;
    private final BStream stalling;

    public Climber(Gamepad gamepad) {

        climber = new WPI_TalonSRX(Ports.Climber.MOTOR);
        Motors.CLIMBER.configure(climber);

        this.speed = 
                IStream.create(() -> gamepad.getLeftY())
                    .filtered(
                        x -> SLMath.deadband(x, Settings.Climber.SPEED_DEADBAND.get()),
                        new LowPassFilter(Settings.Climber.SPEED_RC.get()));
        stalling = 
                BStream.create(() -> isStalling())
                        .filtered(new BDebounceRC.Both(Stalling.DEBOUNCE_TIME.get()));

    }

    public void setMotor(double speed) {
        if (speed != 0.0 && stalling.get()) {
            DriverStation.reportError(
                    "[CRITICAL] Climber is stalling when attempting to move!", false);
            stop();
        } else {
            climber.set(speed);
        }
    }

    public void stop() {
        climber.stopMotor();
    }

    /*** STALL PROTECTION ***/

    private double getDutyCycle() {
        return climber.get();
    }

    private double getCurrentAmps() {
        return Math.abs(climber.getSupplyCurrent());
    }

    public boolean isStalling() {
        boolean current = getCurrentAmps() > Stalling.CURRENT_THRESHOLD;
        boolean output = Math.abs(getDutyCycle()) > Stalling.DUTY_CYCLE_THRESHOLD.get();
        return Stalling.ENABLED.get() && output && current;
    }

    @Override
    public void periodic() {
        if (stalling.get()) {
            DriverStation.reportError("[CRITICAL] Climber is stalling.", false);
            stop();
        } else {
            setMotor(speed.get());
        }

        SmartDashboard.putBoolean("Climber/Stalling", stalling.get());
        SmartDashboard.putNumber("Climber/Current Amps", getCurrentAmps());
        SmartDashboard.putNumber("Climber/Speed", getDutyCycle());
    }
}
