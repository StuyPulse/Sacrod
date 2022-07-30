/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.constants.Settings.Intake;
import com.stuypulse.robot.constants.Settings.RobotSim;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  /** ROBOT SIMULATION */
  Mechanism2d intakeMechanism;
  MechanismRoot2d intakeRoot;
  MechanismLigament2d intake;

  private RobotContainer robot;
  private Command auto;

  /*************************/
  /*** ROBOT SCHEDULEING ***/
  /*************************/

  @Override
  public void robotInit() {
    robot = new RobotContainer();

    /** INTAKE SIMULATOR */
    intakeMechanism = new Mechanism2d(RobotSim.MECHANISM_DIM.x, RobotSim.MECHANISM_DIM.y);
    intakeRoot = intakeMechanism.getRoot("Root", RobotSim.ROOT.x, RobotSim.ROOT.y);
    intake = intakeRoot.append(
        new MechanismLigament2d("Intake", RobotSim.INTAKE_LENGTH, Intake.RETRACT_ANGLE.doubleValue()));

    /** LOGGING */
    SmartDashboard.putData("Intake", intakeMechanism);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    intake.setAngle(robot.intake.getAngle() + 100);
  }

  /*********************/
  /*** DISABLED MODE ***/
  /*********************/

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /***********************/
  /*** AUTONOMOUS MODE ***/
  /***********************/

  @Override
  public void autonomousInit() {
    auto = robot.getAutonomousCommand();

    if (auto != null) {
      auto.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  /*******************/
  /*** TELEOP MODE ***/
  /*******************/

  @Override
  public void teleopInit() {
    if (auto != null) {
      auto.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  /*****************/
  /*** TEST MODE ***/
  /*****************/

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}