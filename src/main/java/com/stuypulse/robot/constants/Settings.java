/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {

    public interface Shooter{
        public interface ShooterPID {

            double kP = 0;
            double kI = 0;
            double kD = 0;

            PIDController PID = new PIDController(kP, kI, kD);

        }
        
        public interface ShooterFF {

            double kS = 0;
            double kV = 0;
            double kA = 0;

            SimpleMotorFeedforward FF = new SimpleMotorFeedforward(kS, kV, kA);

        }
        
        public interface FeederPID {

            double kP = 0;
            double kI = 0;
            double kD = 0;
            
            PIDController PID = new PIDController(kP, kI, kD);
          
        }
        
        public interface FeederFF{

            double kS = 0;
            double kV = 0;
            double kA = 0;

            double FEEDER_RPM_MULTIPLIER = 0.9;

            SimpleMotorFeedforward FF = new SimpleMotorFeedforward(kS, kV, kA);
            
        }
    }
}
