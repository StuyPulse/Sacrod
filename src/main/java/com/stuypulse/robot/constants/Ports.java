/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.robot.constants.Settings.Swerve.Chassis;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.SmartAngle;

import edu.wpi.first.math.geometry.Translation2d;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {

    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface Conveyor {
        int MOTOR = 30;

        int INTAKE_IR = 8;
        int SHOOTER_IR = 9;
    }
    
    public interface Climber {
        int MOTOR = 50;
    }

    public interface Intake {
        int DRIVER_MOTOR = 40;
        int DEPLOYMENT_MOTOR = 41;
    }

    public interface Shooter {
        int SHOOTER_MOTOR = 20;
        int SHOOTER_FOLLOWER = 21;
        int FEEDER_MOTOR = 22;
    }

    public interface Swerve {
        public interface FrontRight {
            String ID = "Front Right";
            int DRIVE = 10;
            int PIVOT = 11;

            int ENCODER = 1;

            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(143));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
        }

        public interface FrontLeft  {
            String ID = "Front Left";
            int DRIVE = 12;
            int PIVOT = 13;

            int ENCODER = 3;

            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(36));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            int DRIVE_PORT = 14;
            int TURN_PORT = 15;
            int ENCODER_PORT = 2;
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(-80.5));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
        }
        
        public interface BackRight {
            String ID = "Back Right";
            int DRIVE_PORT = 16;
            int TURN_PORT = 17;
            int ENCODER_PORT = 0;
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(142.3));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
        }
    }
}
