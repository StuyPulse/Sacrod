/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Intake {

        SmartNumber EXTEND_ANGLE = new SmartNumber("Intake/Extend Angle", 100);
        SmartNumber RETRACT_ANGLE = new SmartNumber("Intake/Retract Angle", 0);

        SmartNumber ACQUIRE_SPEED = new SmartNumber("Intake/Acquire Speed", 1);
        SmartNumber DEACQUIRE_SPEED = new SmartNumber("Intake/Deacquire Speed", -1);

        double GEAR_RATIO = 1. / 28;

        double POSITION_CONVERSION = GEAR_RATIO * 360;

        public interface Deployment {
            SmartNumber MAX_ERROR = new SmartNumber("Intake/Deployment/Max Error", 0.5);

            SmartNumber P = new SmartNumber("Intake/Deployment/P", 0);
            SmartNumber I = new SmartNumber("Intake/Deployment/I", 0);
            SmartNumber D = new SmartNumber("Intake/Deployment/D", 0);

            static Controller getController() {
                return new PIDController(P, I, D);
            }
        }
    }
}
