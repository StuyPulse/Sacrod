/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeExtend;
import com.stuypulse.robot.commands.intake.IntakeRetract;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.climber.SimClimber;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.climber.Climber;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.input.gamepads.keyboard.SimKeyGamepad;
import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

public class RobotContainer {
  private static Gamepad getGamepad(int port) {
    return RobotBase.isReal() ? new AutoGamepad(port) : new SimKeyGamepad();
  }

  // Subsystem
  // public final Conveyor conveyor = new Conveyor();
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final Climber climber = new Climber();
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

  SmartBoolean resetIntake = new SmartBoolean("Intake/Reset Me", false);

  private void configureButtonBindings() {
    driver.getRightTriggerButton()
      .whenPressed(new IntakeExtend(intake))
      .whileHeld(new IntakeAcquire(intake))
      .whenReleased(new IntakeRetract(intake));

    driver.getLeftTriggerButton()
      .whileHeld(new IntakeDeacquire(intake));

    new Button(resetIntake::get).whenPressed(new InstantCommand(() -> intake.reset(0.0), intake));
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
