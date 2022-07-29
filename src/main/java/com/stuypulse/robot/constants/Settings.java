/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Swerve {
        Translation2d SIZE = new Translation2d(Units.inchesToMeters(29.0), Units.inchesToMeters(29.0));
        
        double MAX_SPEED = Units.feetToMeters(14.420); 
        double MAX_ANGULAR_SPEED = MAX_SPEED / (Math.pow(SIZE.getNorm(), 2));

        public interface Controls {
            public interface Turn {
                SmartNumber DEADBAND = new SmartNumber("Controls/Turn/Deadband", 0.05);
                SmartNumber RC = new SmartNumber("Controls/Turn/Smoothing", 0.05);
                SmartNumber POWER = new SmartNumber("Controls/Turn/Power", 2);
            }

            public interface Drive {
                SmartNumber DEADBAND = new SmartNumber("Controls/Drive/Deadband", 0.05);
                SmartNumber RC = new SmartNumber("Controls/Drive/Smoothing", 0.05);
                SmartNumber POWER = new SmartNumber("Controls/Drive/Power", 2);
            }
        }

        public interface DriveControl {
            public interface Encoder {
                double GEARING = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0);
                double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                
                double POSITION_CONVERSION = GEARING * WHEEL_CIRCUMFERENCE;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
            public interface Feedback {
                SmartNumber kP = new SmartNumber("Swerve/Drive/P", 1.0);
                SmartNumber kI = new SmartNumber("Swerve/Drive/I", 0.0);
                SmartNumber kD = new SmartNumber("Swerve/Drive/D", 0.0);

                public static Controller getFeedback() {
                    return new PIDController(kP, kI, kD);
                } 
            }

            public interface FeedForward {
                double kS = 0.11004;
                double kV = 2.7859;
                double kA = 0.10702;

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
                    return ((input - MIN_INPUT) / (MAX_INPUT - MIN_INPUT)) * (MAX_OUTPUT - MIN_OUTPUT) + MIN_OUTPUT;
                }
            }

            public interface Feedback {
                SmartNumber kP = new SmartNumber("Swerve/Turn/P", 2.0);
                SmartNumber kI = new SmartNumber("Swerve/Turn/I", 0.0);
                SmartNumber kD = new SmartNumber("Swerve/Turn/D", 0.0);

                public static AngleController getFeedback() {
                    return new PIDController(kP, kI, kD).angle().useRadians();
                }
            }
        }
    }

}
