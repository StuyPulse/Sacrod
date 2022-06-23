/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.stuypulse.robot.util.SmartPIDController;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public final class Settings {
    public static SmartBoolean DEBUG_MODE = new SmartBoolean("Debug Mode", true);

    public interface Shooter {

        double INTEGRAL_MAX_RPM_ERROR = 500;
        double INTEGRAL_MAX_ADJUST = 1.0;

        double MIN_RPM = 100.0;
        double MAX_TARGET_RPM_CHANGE = 2000.0;

        double MIN_PID_OUTPUT = 0.0;
        double MAX_PID_OUTPUT = 1.0;

        double MAX_RPM_ERROR = 0;
        SmartNumber CHANGE_RC = new SmartNumber("Shooter/Change RC", 0.2);
        SmartNumber FEEDER_MULTIPLIER = new SmartNumber("Shooter/Feeder Multiplier", 0);

        public interface ShooterFF {
            double kS = 0;
            double kV = 0;
            double kA = 0;

            static SimpleMotorFeedforward getController() {
                return new SimpleMotorFeedforward(kS, kV, kA);
            }
        }

        public interface ShooterPID {
            double kP = 0;
            double kI = 0;
            double kD = 0;

            static Controller getController() {
                return new SmartPIDController("Shooter/Shooter")
                        .setControlSpeed(2)
                        .setPID(kP, kI, kD)
                        .setIntegratorFilter(INTEGRAL_MAX_RPM_ERROR, INTEGRAL_MAX_ADJUST);
            }
        }

        public interface FeederFF {
            double kS = 0;
            double kV = 0;
            double kA = 0;

            static SimpleMotorFeedforward getController() {
                return new SimpleMotorFeedforward(kS, kV, kA);
            }
        }

        public interface FeederPID {
            double kP = 0;
            double kI = 0;
            double kD = 0;

            static Controller getController() {
                return new SmartPIDController("Shooter/Feeder")
                        .setControlSpeed(2)
                        .setPID(kP, kI, kD)
                        .setIntegratorFilter(INTEGRAL_MAX_RPM_ERROR, INTEGRAL_MAX_ADJUST);
            }
        }
    }
}
