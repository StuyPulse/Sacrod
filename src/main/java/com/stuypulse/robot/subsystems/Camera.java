package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase{
    public Camera(){
    
    }

    //return tx from the network table
    public Angle getHorizontalOffset(){
        double txDegrees = 10.0 /* read form network table */;
        Angle txAngle = Angle.fromDegrees(txDegrees);
        return txAngle; 
        /* 
        txAngle.toDegrees() -> gets back degree values
        txAngle.toRadians() -> gets back radian values
        */
    }
    
    //return ty from the network table
    private Angle getVerticalOffset(){
        return null;
    }
}
