/************************ PROJECT SACROD ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.climber.*;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveGyroFeedback;
import com.stuypulse.robot.commands.swerve.SwerveDriveHome;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.shooter.*;
import com.stuypulse.robot.util.ConveyorMode;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.Xbox;
import com.stuypulse.stuylib.input.gamepads.keyboard.SimKeyGamepad;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private static Gamepad getGamepad(int port) {
    return new Xbox(port);
  }

  // Subsystem
  public final Conveyor conveyor = new Conveyor();
  public final IShooter shooter = IShooter.getInstance();
  public final IIntake intake = IIntake.getInstance();
  public final IClimber climber = IClimber.getInstance();
  public final SwerveDrive swerve = SwerveDrive.getInstance();

  public final Camera camera = new Camera();

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
    // swerve.setDefaultCommand(new SwerveDriveDrive(swerve, driver));
    swerve.setDefaultCommand(new SwerveDriveDriveGyroFeedback(swerve, driver));
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    driver.getTopButton().whenPressed(new SwerveDriveHome(swerve));

    operator.getLeftTriggerButton()
      .whileHeld(new IntakeDeacquire(intake))
      .whileHeld(new ConveyorSetMode(conveyor, ConveyorMode.REVERSE));

    operator.getRightTriggerButton()
      .whileHeld(new IntakeAcquire(intake))
      .whenPressed(new IntakeExtend(intake))
      .whenReleased(new IntakeRetract(intake));

    operator.getTopButton()
      .whenPressed(new ConveyorSetMode(conveyor, ConveyorMode.INDEXING));  
    operator.getRightButton()
      .whileHeld(new ConveyorSetMode(conveyor, ConveyorMode.SHOOTING));
    operator.getLeftButton()
      .whileHeld(new ConveyorSetMode(conveyor, ConveyorMode.BRING_UP_BALLS));

    operator.getDPadUp().whenPressed(new ShooterStop(shooter));
    operator.getDPadRight().whenPressed(new ShooterSetRPM(shooter, Settings.Scoring.PRIMARY_RPM));
    operator.getDPadLeft().whenPressed(new ShooterSetRPM(shooter, Settings.Scoring.SECONDARY_RPM));
    operator.getDPadDown().whenPressed(new ShooterSetRPM(shooter, Settings.Scoring.TUNING_RPM));
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
