package com.stuypulse.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.filters.RateLimit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final IFilter targetFilter;
    private final PIDFlywheel shooter;
    private final PIDFlywheel feeder;

    private final SmartNumber targetRPM;

    public Shooter() {
        targetRPM = new SmartNumber("Shooter/Target RPM", 0.0);

        targetFilter = new RateLimit(Settings.Shooter.MAX_TARGET_RPM_CHANGE)
                .then(new LowPassFilter(Settings.Shooter.CHANGE_RC));

        CANSparkMax shooterMotor = new CANSparkMax(Ports.Shooter.LEFT, MotorType.kBrushless);
        CANSparkMax shooterFollower = new CANSparkMax(Ports.Shooter.RIGHT, MotorType.kBrushless);

        shooter = new PIDFlywheel(
                shooterMotor,
                Settings.Shooter.ShooterFF.getController(),
                Settings.Shooter.ShooterPID.getController());
        shooter.addFollower(shooterFollower);

        CANSparkMax feederMotor = new CANSparkMax(Ports.Shooter.FEEDER, MotorType.kBrushless);

        feeder = new PIDFlywheel(
                feederMotor,
                Settings.Shooter.FeederFF.getController(),
                Settings.Shooter.FeederPID.getController());
    }

    /** Shooter controls */
    public void setShooterRPM(Number speed) {
        targetRPM.set(speed);
    }

    /** Encoder readings */
    public double getShooterRPM() {
        return shooter.getVelocity();
    }

    public double getFeederRPM() {
        return feeder.getVelocity();
    }

    /*** TARGET RPM READING ***/

    public double getRawTargetRPM() {
        return targetRPM.get();
    }

    public double getTargetRPM() {
        return targetFilter.get(getRawTargetRPM());
    }

    public boolean isReady() {
        return Math.abs(getShooterRPM() - getRawTargetRPM()) < Settings.Shooter.MAX_RPM_ERROR;
    }

    @Override
    public void periodic() {
        double setpoint = getTargetRPM();

        if (setpoint < Settings.Shooter.MIN_RPM) {
            shooter.stop();
            feeder.stop();
        } else {
            shooter.setVelocity(setpoint);
            feeder.setVelocity(setpoint * Settings.Shooter.FEEDER_MULTIPLIER.get());
        }

        if (Settings.DEBUG_MODE.get()) {
            SmartDashboard.putNumber("Debug/Shooter/Shooter RPM", getShooterRPM());
            SmartDashboard.putNumber("Debug/Shooter/Feeder RPM", getFeederRPM());
        }
    }

}