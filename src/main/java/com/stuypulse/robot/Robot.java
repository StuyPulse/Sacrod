/************************ PROJECT SACROD ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.ConveyorMode;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  public static final com.stuypulse.robot.constants.Settings.Robot SACROD = null;
  private RobotContainer robot;
  private Command auto;

  /*************************/
  /*** ROBOT SCHEDULEING ***/
  /*************************/

  @Override
  public void robotInit() {
    DataLogManager.start();

    robot = new RobotContainer();
  }

  @Override
  public void robotPeriodic() { 
    CommandScheduler.getInstance().run();
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

    new SwerveDriveResetHeading().schedule();

    if (auto != null) {
      auto.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    SwerveDrive.getInstance().setXMode();
    Shooter.getInstance().stop();
  }

  /*******************/
  /*** TELEOP MODE ***/
  /*******************/

  @Override
  public void teleopInit() {
    if (auto != null) {
      auto.cancel();
    }

    new ConveyorSetMode(ConveyorMode.DEFAULT);

    Intake.getInstance().retract();
    Intake.getInstance().stop();
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
