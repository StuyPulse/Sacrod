/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {

    public interface Shooter{
        
        public interface ShooterPID {

            SmartNumber kP = new SmartNumber("Shooter/Shooter kP", 0);
            SmartNumber kI = new SmartNumber("Shooter/Shooter kI", 0);
            SmartNumber kD = new SmartNumber("Shooter/Shooter kD", 0);

            static PIDController PID(){
                return new PIDController(kP, kI, kD);
            }
        }
        
        public interface ShooterFF {

            double kS = 0;
            double kV = 0;
            double kA = 0;

            static SimpleMotorFeedforward FF(){
                return new SimpleMotorFeedforward(kS, kV, kA);
            }

        }
        
        public interface FeederPID {

            SmartNumber kP = new SmartNumber("Shooter/Feeder kP", 0);
            SmartNumber kI = new SmartNumber("Shooter/Feeder kP", 0);
            SmartNumber kD = new SmartNumber("Shooter/Feeder kP", 0);
            
            static PIDController PID(){
                return new PIDController(kP, kI, kD);
            }
          
        }
        
        public interface FeederFF{

            double kS = 0;
            double kV = 0;
            double kA = 0;

            double FEEDER_RPM_MULTIPLIER = 0.9;

            static SimpleMotorFeedforward FF(){
                return new SimpleMotorFeedforward(kS, kV, kA);
            }
        }
    }
}
