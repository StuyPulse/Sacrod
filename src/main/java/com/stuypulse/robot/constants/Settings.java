/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.math.interpolation.Interpolator;
import com.stuypulse.stuylib.math.interpolation.LinearInterpolator;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {

   public interface Climber {
        
        SmartNumber DEADBAND = new SmartNumber("Climber/Deadband", 0.05);
        SmartNumber RC = new SmartNumber("Climber/RC", 0.05);

        public interface Feedback {
            SmartNumber kP = new SmartNumber("Climber/P", 0.0);
            SmartNumber kI = new SmartNumber("Climber/I", 0.0);
            SmartNumber kD = new SmartNumber("Climber/D", 0.0);
        }
        
        double MIN_HEIGHT = 0.0;
        double MAX_HEIGHT = 1.0;

        public interface Encoder {
            int COUNTERS_PER_REVOLUTION = 1024;
            double GEAR_RATIO = 9.0 / 1.0;
            double CONVERSION_FACTOR = 1.0;
        }

    }
  
    public interface Conveyor {
        SmartNumber FORWARD_SPEED = new SmartNumber("Conveyor/Forward Speed", 1.0);
        SmartNumber REVERSE_SPEED = new SmartNumber("Conveyor/Reverse Speed", -1.0);
    }
    
    public interface Shooter{
        
        public interface ShotMap {
            /*
             TODO: 
              - find ring shot distance
              - find (farthest) distance to driver station wall
              - determine how many points we want to tune for
              - calculate what distances we're tuning RPMS for
            */
            Interpolator DISTANCE_TO_RPM = new LinearInterpolator(
                new Vector2D(0.0, 0.0)
            );
        }

        SmartNumber RING_RPM = new SmartNumber("Shooter/Ring RPM", 3000);

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

            SmartNumber kP = new SmartNumber("Intake/Deployment/P", 0.01);
            SmartNumber kI = new SmartNumber("Intake/Deployment/I", 0);
            SmartNumber kD = new SmartNumber("Intake/Deployment/D", 0);

            static Controller getController() {
                return new PIDController(kP, kI, kD);
            }
        }
    }
}
