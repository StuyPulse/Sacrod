/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
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

    public interface Conveyor {
        SmartNumber FORWARD_SPEED = new SmartNumber("Conveyor/Forward Speed", 1.0);
        SmartNumber REVERSE_SPEED = new SmartNumber("Conveyor/Reverse Speed", -1.0);
    }
    
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

    public interface Intake {

        SmartNumber EXTEND_ANGLE = new SmartNumber("Intake/Extend Angle", 100);
        SmartNumber RETRACT_ANGLE = new SmartNumber("Intake/Retract Angle", 0);

        SmartNumber ACQUIRE_SPEED = new SmartNumber("Intake/Acquire Speed", 1);
        SmartNumber DEACQUIRE_SPEED = new SmartNumber("Intake/Deacquire Speed", -1);

        double GEAR_RATIO = 1. / 28;

        double POSITION_CONVERSION = GEAR_RATIO * 360;

        public interface Deployment {
            SmartNumber MAX_ERROR = new SmartNumber("Intake/Deployment/Max Error", 3.0);

            SmartNumber P = new SmartNumber("Intake/Deployment/P", 0.01);
            SmartNumber I = new SmartNumber("Intake/Deployment/I", 0);
            SmartNumber D = new SmartNumber("Intake/Deployment/D", 0);

            static Controller getController() {
                return new PIDController(P, I, D);
            }
        }
    }
}
