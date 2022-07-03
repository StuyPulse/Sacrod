/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.control.angle.AngleController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Swerve {

        public interface DriveControl {
            public interface Encoder {
                double GEARING = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);
                double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                
                double POSITION_CONVERSION = GEARING * WHEEL_CIRCUMFERENCE;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
            public interface Feedback {
                double kP = 0.0;
                double kI = 0.0;
                double kD = 0.0;

                public static Controller getFeedback() {
                    return new PIDController(kP, kI, kD);
                } 
            }

            public interface FeedForward {
                double kS = 0.0;
                double kV = 0.0;
                double kA = 0.0;

                public static SimpleMotorFeedforward getFeedForward() {
                    return new SimpleMotorFeedforward(kS, kV, kA);
                }
            }
        }

        public interface TurnControl {

            public interface Encoder {
                double MIN_INPUT = 0.0;
                double MAX_INPUT = 1.0;

                double MIN_OUTPUT = -Math.PI;
                double MAX_OUTPUT = +Math.PI;

                static double getRadians(double input) {
                    return ((input - MIN_INPUT) / MIN_OUTPUT) * (MAX_OUTPUT - MIN_OUTPUT) + MIN_OUTPUT;
                }                
            }

            public interface Feedback {
                double kP = 0.0;
                double kI = 0.0;
                double kD = 0.0;

                public static AngleController getFeedback() {
                    return new PIDController(kP, kI, kD).angle();
                }
            }
        }
    }

}
