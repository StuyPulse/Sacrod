/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public interface Climber {

        SmartNumber NORMAL_SPEED = new SmartNumber("Climber/Speed", 1);

        SmartNumber SPEED_DEADBAND = new SmartNumber("Climber/Speed Deadband", 0.001);
        SmartNumber SPEED_RC = new SmartNumber("Climber/Speed RC", 0.2);
        
        public interface Stalling {
            SmartBoolean ENABLED = new SmartBoolean("Climber/Stall Detection", false);

            // Motor will hit current limit when stalling
            double CURRENT_THRESHOLD = Motors.CLIMBER.PEAK_CURRENT_LIMIT_AMPS - 10;

            // If we are trying to go at full speed,
            // it doesnt matter if our current draw isnt that high
            double DUTY_CYCLE_THRESHOLD = 0.1;

            double DEBOUNCE_TIME = 1.0;
        }

    }
    
}
