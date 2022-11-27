/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.math.interpolation.Interpolator;
import com.stuypulse.stuylib.math.interpolation.LinearInterpolator;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {
    double DT = 0.02;

    public interface Climber {

        SmartNumber ERROR = new SmartNumber("Climber/Error", 0.01);
        SmartNumber DEADBAND = new SmartNumber("Climber/Deadband", 0.05);
        SmartNumber RC = new SmartNumber("Climber/RC", 0.05);
        SmartNumber MAX_VELOCITY = new SmartNumber("Climber/Max Velocity", 1);

        public interface Feedback {
            SmartNumber kP = new SmartNumber("Climber/P", 1.0 / 48.0);
            SmartNumber kI = new SmartNumber("Climber/I", 0.0);
            SmartNumber kD = new SmartNumber("Climber/D", 0.0);
        }

        public interface Feedforward {
            double kV = 12.0 / (1.0 / 2.0);
            double kA = 12.0 / ((1.0 / 2.0) / 0.05);
        }

        double MIN_HEIGHT = 40.475;
        double MAX_HEIGHT = 64.1;
        // double HOOK_HEIGHT = 3.5 in;
        // double CLIMBER_HEIGHT = 60.6 in to top;

        public interface Encoder {
            int COUNTS_PER_REVOLUTION = 1024;
            double GEAR_RATIO = 25.0 / 1.0; // Encoder is after gearing
            double OUTPUT_TO_SPOOL = 1.0 / 1.0; // There is ratio between gearbox output turns and spool turns

            double SPOOL_DIAMETER = 1.25;
            double SPOOL_CIRCUMFERENCE = SPOOL_DIAMETER * Math.PI; // Distance travelled by spool is the height
                                                                   // travelled by the climber

            double CONVERSION_FACTOR = SPOOL_CIRCUMFERENCE / OUTPUT_TO_SPOOL / COUNTS_PER_REVOLUTION;
        }

    }

    public interface Conveyor {
        SmartNumber FORWARD_SPEED = new SmartNumber("Conveyor/Forward Speed", 1.0);
        SmartNumber REVERSE_SPEED = new SmartNumber("Conveyor/Reverse Speed", -1.0);
    }

    public interface Shooter {

        public interface ShotMap {
            /*
             * TODO:
             * - find ring shot distance
             * - find (farthest) distance to driver station wall
             * - determine how many points we want to tune for
             * - calculate what distances we're tuning RPMS for
             */
            Interpolator DISTANCE_TO_RPM = new LinearInterpolator(
                    new Vector2D(0.0, 0.0));
        }

        SmartNumber RING_RPM = new SmartNumber("Shooter/Ring RPM", 3000);

        public interface ShooterPID {

            // TODO: auto pid tune this
            SmartNumber kP = new SmartNumber("Shooter/Shooter kP", 0.0050642);
            SmartNumber kI = new SmartNumber("Shooter/Shooter kI", 0);
            SmartNumber kD = new SmartNumber("Shooter/Shooter kD", 0);

        }

        public interface ShooterFF {
            SmartNumber kS = new SmartNumber("Shooter/Shooter kS", 0.28656);
            SmartNumber kV = new SmartNumber("Shooter/Shooter kV", 0.0021618);
            SmartNumber kA = new SmartNumber("Shooter/Shooter kA", 0.00012967);
        }

        public interface FeederPID {

            SmartNumber kP = new SmartNumber("Shooter/Feeder kP", 0.008743);
            SmartNumber kI = new SmartNumber("Shooter/Feeder kI", 0);
            SmartNumber kD = new SmartNumber("Shooter/Feeder kD", 0);

        }

        public interface FeederFF {

            SmartNumber kS = new SmartNumber("Shooter/Feeder kS", 0.16826);
            SmartNumber kV = new SmartNumber("Shooter/Feeder kV", 0.0021141);
            SmartNumber kA = new SmartNumber("Shooter/Feeder kA", 0.00020526);

            SmartNumber FEEDER_RPM_MULTIPLIER = new SmartNumber("Shooter/Feeder RPM Multiplier", 1);

        }
    }

    public interface Intake {

        SmartNumber EXTEND_ANGLE = new SmartNumber("Intake/Extend Angle", 99);
        SmartNumber RETRACT_ANGLE = new SmartNumber("Intake/Retract Angle", 0);

        SmartNumber ACQUIRE_SPEED = new SmartNumber("Intake/Acquire Speed", 1);
        SmartNumber DEACQUIRE_SPEED = new SmartNumber("Intake/Deacquire Speed", -1);

        double GEAR_RATIO = 28.0 / 1.0;

        double POSITION_CONVERSION = 360.0 / GEAR_RATIO;
        // double POSITION_CONVERSION = 1.0;

        public interface Deployment {
            SmartNumber MAX_ERROR = new SmartNumber("Intake/Deployment/Max Error", 3.0);

            SmartNumber kP = new SmartNumber("Intake/Deployment/P", 0.012);
            SmartNumber kI = new SmartNumber("Intake/Deployment/I", 0);
            SmartNumber kD = new SmartNumber("Intake/Deployment/D", 0.00035);
        }

        public interface Simulation {
            double MAX_DEGREES_PER_SECOND = 666.36;

            SmartNumber kV = new SmartNumber("Intake/Simulation/V", 12.0 / MAX_DEGREES_PER_SECOND);
            SmartNumber kA = new SmartNumber("Intake/Simulation/A", 0.01);
        }
    }
}
