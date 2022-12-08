/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climber.*;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.shooter.*;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.input.gamepads.Xbox;
import com.stuypulse.stuylib.input.gamepads.keyboard.SimKeyGamepad;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private static Gamepad getGamepad(int port) {
    return new Xbox(port);
  }

  // Subsystem
  // public final Conveyor conveyor = new Conveyor();
  public final IShooter shooter = new SimShooter();
  public final IIntake intake = new SimIntake();
  public final IClimber climber = new SimClimber();
  public final Swerve swerve = new Swerve();

  // Gamepads
  public final Gamepad driver = getGamepad(Ports.Gamepad.DRIVER);
  public final Gamepad operator = getGamepad(Ports.Gamepad.OPERATOR);

  // Autons
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Robot container

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    driver.getRightTriggerButton()
      .whenPressed(new IntakeExtend(intake))
      .whileHeld(new IntakeAcquire(intake))
      .whenReleased(new IntakeRetract(intake));

    driver.getLeftTriggerButton()
      .whileHeld(new IntakeDeacquire(intake));

    operator.getLeftTriggerButton()
      .whenHeld(new IntakeRetract(intake));

    operator.getRightTriggerButton()
      .whenHeld(new IntakeAcquire(intake))
      .whenPressed(new IntakeExtend(intake))
      .whenReleased(new IntakeRetract(intake));

    operator.getTopButton()
      .whenPressed(new ClimberToTop(climber));

    operator.getBottomButton()
      .whenPressed(new ClimberToBottom(climber));
  }

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
