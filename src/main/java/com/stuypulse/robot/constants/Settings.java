/************************ PROJECT SACROD ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.math.interpolation.Interpolator;
import com.stuypulse.stuylib.math.interpolation.LinearInterpolator;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {

    public enum Robot {
        SACROD,
        JIM
    }

    Robot ROBOT = Robot.SACROD;

    double DT = 0.02;

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }

    public static void reportWarning(String warning) {
        DriverStation.reportWarning(warning, false);
    }

    public interface Driver {
        SmartNumber DEADBAND = new SmartNumber("Driver Settings/Deadband", 0.05);
        SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Max Speed", 4); // 4.2
        SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Max Turning", 4); // 6.1
        SmartNumber MAX_ACCELERATION = new SmartNumber("Driver Settings/Max Acceleration", 4); // 5

        public interface Drive {
            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.1);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);
        }

        public interface Turn {
            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.15);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);
        }

    }

    public interface Scoring {
        // SmartNumber PRIMARY_DISTANCE = new SmartNumber("Scoring/Primary Distance",
        // 3.71);
        // TUNE VALUES

        SmartNumber CS_RPM = new SmartNumber("Scoring/CS RPM", 1400);
        SmartNumber FAR_RPM = new SmartNumber("Scoring/Far RPM", 3000); // 1750
        SmartNumber HIGH_RPM = new SmartNumber("Scoring/High RPM", 1000); // 1750
        // SmartNumber MID_DISTANCE = new SmartNumber("Scoring/Secondary Distance",
        // 4.55);
        SmartNumber MID_RPM = new SmartNumber("Scoring/Mid RPM", 750); // 2000
        SmartNumber LOW_RPM = new SmartNumber("Scoring/Low RPM", 500);

        // Interpolator DISTANCE_TO_RPM = new LinearInterpolator(
        // new Vector2D(HIGH_RPM.get(), HIGH_RPM.get()),
        // new Vector2D(MID_RPM.get(), MID_RPM.get()));

        SmartNumber ACCEPTABLE_RPM = new SmartNumber("Scoring/Acceptable RPM Error", 200);
        SmartNumber ACCEPTABLE_VELOCITY = new SmartNumber("Scoring/Acceptable Shot Velocity", Units.inchesToMeters(9));
        SmartNumber ACCEPTABLE_TURN_ERROR = new SmartNumber("Scoring/Acceptable Turn Error (deg)", 7.5);
        SmartNumber ACCEPTABLE_DISTANCE_ERROR = new SmartNumber("Scoring/Acceptable Distance Error (m)",
                Units.inchesToMeters(4));
        SmartNumber READY_TIME = new SmartNumber("Scoring/Ready Time", 0.2);

        // TODO: make shooting and alignment command specifically for testing
        SmartNumber TUNING_RPM = new SmartNumber("Scoring/Tuning RPM", 0.0);
        SmartNumber TUNING_DISTANCE = new SmartNumber("Scoring/Tuning Distance", 0.0);

        public interface AutoShot {
            SmartNumber kP = new SmartNumber("Scoring/Auto Shot/kP", 1.0);
            SmartNumber kI = new SmartNumber("Scoring/Auto Shot/kI", 0.0);
            SmartNumber kD = new SmartNumber("Scoring/Auto Shot/kD", 0.2);

            SmartNumber FUSION_FILTER = new SmartNumber("Scoring/Auto Shot/Fusion RC", 0.3);
            SmartNumber MAX_SPEED = new SmartNumber("Scoring/Auto Shot/Speed", 2);
        }

        public interface Auton {
            SmartNumber kP = new SmartNumber("Scoring/Auton/Turn kP", 1);
            SmartNumber kI = new SmartNumber("Scoring/Auton/Turn kI", 0.0);
            SmartNumber kD = new SmartNumber("Scoring/Auton/Turn kD", 0.5);

            SmartNumber DISTkP = new SmartNumber("Scoring/Auton/Distance kP", 3.0);
            SmartNumber DISTkI = new SmartNumber("Scoring/Auton/Distance kI", 0.0);
            SmartNumber DISTkD = new SmartNumber("Scoring/Auton/Distance kD", 0.2);

            SmartNumber FUSION_FILTER = new SmartNumber("Scoring/Auton/Fusion RC", 0.3);
        }
    }

    public interface Limelight {
        int[] PORTS = { 5800, 5801, 5802, 5803, 5804, 5805 };
        double CAMERA_TO_CENTER = Units.inchesToMeters(14.0);
        Angle CAMERA_PITCH = Angle.fromDegrees(28);
        double CAMERA_HEIGHT = Units.inchesToMeters(32);
    }

    public interface Field {
        double HUB_HEIGHT = Units.feetToMeters(8) + Units.inchesToMeters(9);
        double HUB_TO_CENTER = Units.feetToMeters(2.0);
        Translation2d HUB = new Translation2d(Units.feetToMeters(12.8), 0);
    }
    public interface AutoBalance{ //TUNE THESEE
        SmartNumber DISTANCE_THRESHOLD = new SmartNumber("Auto Balance/Dual PID/Distance Threshold", 0.05);
        SmartNumber ANGLE_THRESHOLD = new SmartNumber("Auto Balance/Dual PID/Angle Thrshold", 10); // 12 originally

        SmartNumber MAX_TILT = new SmartNumber("Auto Balance/Max Tilt (deg)", 15.0);
        SmartNumber MAX_SPEED = new SmartNumber("Auto Balance/Max Engage Speed (m per s)", 0.5);

        SmartNumber kT_u = new SmartNumber("Auto Balance/With Plant/Tu", 0.2);  // from Zieger-Nichols tuning method

        public interface Translation {
            SmartNumber P = new SmartNumber("Auto Balance/Translation/kP", 0.05);
            SmartNumber I = new SmartNumber("Auto Balance/Translation/kI", 0);
            SmartNumber D = new SmartNumber("Auto Balance/Translation/kD", 0);
        }

        public interface Tilt {
            SmartNumber P = new SmartNumber("Auto Balance/Tilt/kP", 0.05);
            SmartNumber I = new SmartNumber("Auto Balance/Tilt/kI", 0);
            SmartNumber D = new SmartNumber("Auto Balance/Tilt/kD", 0);
        }

        public interface Gyro {
            SmartNumber kT_u = new SmartNumber("Auto Engage/Tu", 0.2);  // from Zieger-Nichols tuning method
            Number kK_u = IStream.create(() -> MAX_SPEED.get() / MAX_TILT.get()).number();  // from Zieger-Nichols tuning method

            Number kP = IStream.create(() -> 0.8 * kK_u.doubleValue()).number();  // from Zieger-Nichols tuning method
            SmartNumber kI = new SmartNumber("", 0);
            Number kD = IStream.create(() -> 0.1 * kK_u.doubleValue() * kT_u.doubleValue()).number(); // from Zieger-Nichols tuning method
        }
    }

    public interface Climber {

        SmartNumber ERROR = new SmartNumber("Climber/Error", 0.01);
        SmartNumber DEADBAND = new SmartNumber("Climber/Deadband", 0.05);
        SmartNumber RC = new SmartNumber("Climber/RC", 0.05);
        SmartNumber MAX_VELOCITY = new SmartNumber("Climber/Max Velocity", 1);

        public interface Feedback {
            SmartNumber kP = new SmartNumber("Climber/P", 0.5);
            SmartNumber kI = new SmartNumber("Climber/I", 0.0);
            SmartNumber kD = new SmartNumber("Climber/D", 0.0);
        }

        public interface Feedforward {
            double kV = 12.0 / 3.0;
            double kA = 12.0 / (3.0 / 0.05);
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
        SmartNumber FORWARD_SPEED = new SmartNumber("Conveyor/Forward Speed", 0.7);
        SmartNumber REVERSE_SPEED = new SmartNumber("Conveyor/Reverse Speed", -0.7);

        SmartNumber EMPTY_DEBOUNCE = new SmartNumber("Conveyor/Is Empty Debounce", 0.2);
        SmartNumber HASBALL_DEBOUNCE = new SmartNumber("Conveyor/Has Ball Debounce", 0.2);
    }

    public interface Arm {
        // REPLACE VALUES
        SmartNumber ARM_EXTEND_ANGLE = new SmartNumber("Arm/Extend Angle", 0);
        SmartNumber ARM_RETRACT_ANGLE = new SmartNumber("Arm/Retract Angle", 0);

        public interface Deployment {
            // CHANGE VALUES
            SmartNumber MAX_ERROR = new SmartNumber("Arm/Deployment/Max Error", 3.0);

            SmartNumber kP = new SmartNumber("Arm/Deployment/P", 0);
            SmartNumber kI = new SmartNumber("Ar/Deployment/I", 0);
            SmartNumber kD = new SmartNumber("Arm/Deployment/D", 0);
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
            double MAX_DEGREES_PER_SECOND = 2000.0;

            SmartNumber kV = new SmartNumber("Intake/Simulation/V", 12.0 / MAX_DEGREES_PER_SECOND);
            SmartNumber kA = new SmartNumber("Intake/Simulation/A", 0.01);
        }
    }

    public interface Shooter {
        public interface ShooterPID {
            // TODO: auto pid tune this
            SmartNumber kP = new SmartNumber("Shooter/Shooter kP", 0); // 0.0050642);
            SmartNumber kI = new SmartNumber("Shooter/Shooter kI", 0);
            SmartNumber kD = new SmartNumber("Shooter/Shooter kD", 0);
        }

        public interface ShooterFF {
            SmartNumber kS = new SmartNumber("Shooter/Shooter kS", 0.28656);
            SmartNumber kV = new SmartNumber("Shooter/Shooter kV", 0.0021618);
            SmartNumber kA = new SmartNumber("Shooter/Shooter kA", 0.00012967);
        }

        public interface FeederPID {
            SmartNumber kP = new SmartNumber("Shooter/Feeder kP", 0); // 0.008743);
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

    public interface Swerve {
        double MIN_MODULE_VELOCITY = 0.05;

        public interface Chassis {
            double WIDTH = Units.inchesToMeters(29.0);
            double HEIGHT = Units.inchesToMeters(29.0);
            double MAX_SPEED = 4.2;
        }

        public interface Drive {
            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.17335);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.7274);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.456);

            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 0.7);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.05);
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.1);

            double kS = 0.14;
            double kV = 0.25;
            double kA = 0.007;
        }

        public interface FrontRight {
            String ID = "Front Right";
            SmartNumber ABSOLUTE_OFFSET = new SmartNumber("Swerve/" + ID + "/Absolute Offset", -39.2 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            SmartNumber ABSOLUTE_OFFSET = new SmartNumber("Swerve/" + ID + "/Absolute Offset", 25.3);
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            SmartNumber ABSOLUTE_OFFSET = new SmartNumber("Swerve/" + ID + "/Absolute Offset", -80.1);
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            SmartNumber ABSOLUTE_OFFSET = new SmartNumber("Swerve/" + ID + "/Absolute Offset", -37.0 + 180);
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

                public interface Stages {
                    // input / output
                    double FIRST = 16.0 / 48.0;
                    double SECOND = 28.0 / 16.0;
                    double THIRD = 15.0 / 60.0;
                }

                double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double GEAR_RATIO = 1.0 / 12.8;
                double POSITION_CONVERSION = GEAR_RATIO * 2 * Math.PI;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }

        public interface Motion {
            PathConstraints CONSTRAINTS = new PathConstraints(3.5, 2);

            PIDConstants XY = new PIDConstants(0.4, 0, 0.04);
            PIDConstants THETA = new PIDConstants(0.2, 0, 0.02);

        }
    }
}
