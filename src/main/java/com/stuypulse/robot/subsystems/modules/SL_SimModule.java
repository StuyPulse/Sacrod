package com.stuypulse.robot.subsystems.modules;

import com.stuypulse.robot.constants.Settings;

import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
// import com.stuypulse.robot.util.DelayFilter;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.*;
import com.stuypulse.stuylib.control.feedforward.Feedforward;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SL_SimModule extends SwerveModule {
   /** MODULE **/
    
   private final String id;
   private final Translation2d moduleOffset;
   
   private SwerveModuleState targetState;

   /** TURNING **/
   
   private final LinearSystemSim<N2, N1, N1> turnSim;
   private double turnVoltage;

   private final AngleController turnController;
   
   /** DRIVING */
   
   private final LinearSystemSim<N1, N1, N1> driveSim;
   private double driveVoltage;

   private final Controller driveController;

   public SL_SimModule(String id, Translation2d moduleOffset) {
       // Module
       this.id = id;
       this.moduleOffset = moduleOffset;

       targetState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

       // Turning
       turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV, Turn.kA));
       turnVoltage = 0.0;
       
       turnController = new AnglePIDController(Turn.kP, Turn.kI, Turn.kD)
           // .add(new Feedforward.Motor(Turn.kS, Turn.kV, Turn.kA).angle())
       ;

       // Driving
       driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(Drive.kV.get(), Drive.kA.get()));
       driveVoltage = 0.0;

       driveController = new PIDController(Drive.kP, Drive.kI, Drive.kD)
           .add(new Feedforward.Motor(Drive.kS, Drive.kV, Drive.kA).velocity());
       
   }

   /** MODULE METHODS **/
   
   public String getId() {
       return id;
   }

   public Translation2d getLocation() {
       return moduleOffset;
   }

   public void setTargetState(SwerveModuleState state) {
        targetState = SwerveModuleState.optimize(state, getAngle().getRotation2d());
   }

   public SwerveModuleState getState() {
       return new SwerveModuleState(getSpeed(), getAngle().getRotation2d());
   }
   
   /** TURNING METHODS **/

   public Angle getAngle() {
       return Angle.fromRadians(turnSim.getOutput(0));
   }

   /** DRIVING METHODS **/
   
   public double getSpeed() {
       return driveSim.getOutput(0);
   }
   
   /** CONTROL LOOP **/
   @Override
   public void periodic() {

       // Control Loops
       turnVoltage = turnController.update(Angle.fromRotation2d(targetState.angle), getAngle());
       driveVoltage = driveController.update(targetState.speedMetersPerSecond, getSpeed());

       // Network Logging
       SmartDashboard.putNumber("Swerve/" + id + "/Angle", MathUtil.inputModulus(getAngle().toDegrees(), -180, +180));
       SmartDashboard.putNumber("Swerve/" + id + "/Speed", getSpeed());
       
       SmartDashboard.putNumber("Swerve/" + id + "/Target Angle", MathUtil.inputModulus(targetState.angle.getDegrees(), -180, +180));
       SmartDashboard.putNumber("Swerve/" + id + "/Target Speed", targetState.speedMetersPerSecond);
   }

   @Override
   public void simulationPeriodic() {
       // Drive Simulation
       driveSim.setInput(driveVoltage);

       driveSim.update(Settings.DT);

       // Turn Simulation
       turnSim.setInput(turnVoltage);

       turnSim.update(Settings.DT);

       // Robot Simulation
       RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
           turnSim.getCurrentDrawAmps() + driveSim.getCurrentDrawAmps()
       ));
   }
}
