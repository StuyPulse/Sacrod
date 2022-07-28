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
        int MOTOR = -1;
        int INTAKE_IR = -1;
        int SHOOTER_IR = -1;
    }
    
    public interface Climber {
        int MOTOR = 10;
    }

    public interface Intake {
        int DRIVER_MOTOR = 0;
        int DEPLOYMENT_MOTOR = 1;
    }

    public interface Shooter {
        int SHOOTER_MOTOR = 2;
        int SHOOTER_FOLLOWER = 3;
        int FEEDER_MOTOR = 4;
    }
}
