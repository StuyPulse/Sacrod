/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

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
}
