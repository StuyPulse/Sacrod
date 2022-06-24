package com.stuypulse.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Climber.Stalling;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*-
 * @author Ivan Chen
 */

public class Climber extends SubsystemBase {
    
    private final WPI_TalonSRX climber;

    private final Debouncer stalling;

    public Climber() {
        climber = new WPI_TalonSRX(Ports.Climber.MOTOR);
        Motors.CLIMBER.configure(climber);

        stalling = new Debouncer(Stalling.DEBOUNCE_TIME, DebounceType.kBoth);
    }

    public void setMotor(double speed) {
        if (speed != 0.0 && isStalling()) {
            DriverStation.reportError(
                    "[CRITICAL] Climber is stalling when attempting to move!", false);
            stalling.calculate(true);
            stop();
        } else {
            climber.set(speed);
        }
    }

    public void forceLowerClimber() {
        climber.set(-Settings.Climber.SLOW_SPEED.get());
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
        boolean output = Math.abs(getDutyCycle()) > Stalling.DUTY_CYCLE_THRESHOLD;
        return Stalling.ENABLED.get() && stalling.calculate(output && current);
    }

    @Override
    public void periodic() {
        if (isStalling()) {
            DriverStation.reportError("[CRITICAL] Climber is stalling.", false);
            stop();
        }

        SmartDashboard.putBoolean("Climber/Stalling", isStalling());
        SmartDashboard.putNumber("Climber/Current Amps", getCurrentAmps());
        SmartDashboard.putNumber("Climber/Speed", getDutyCycle());
    }
}
