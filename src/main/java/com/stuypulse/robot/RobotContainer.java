/************************ PROJECT SACROD ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.arm.ArmDown;
import com.stuypulse.robot.commands.arm.ArmUp;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.MobilityAuton;
import com.stuypulse.robot.commands.auton.MylesAuto;
import com.stuypulse.robot.commands.auton.OnePieceDock;
import com.stuypulse.robot.commands.auton.OnePieceMobilityDock;
import com.stuypulse.robot.commands.auton.OnePieceMobilityNonwire;
import com.stuypulse.robot.commands.auton.OnePieceMobilityWire;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.camera.Camera;
import com.stuypulse.robot.subsystems.shooter.*;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.robot.util.ConveyorMode;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.robot.commands.shooter.*;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // Subsystem
  public final SwerveDrive swerve = SwerveDrive.getInstance();
  public final Conveyor conveyor = Conveyor.getInstance();
  public final Shooter shooter = Shooter.getInstance();
  public final Intake intake = Intake.getInstance();
  public final Climber climber = Climber.getInstance();

  public final Camera camera = Camera.getInstance();

  // Gamepads
  public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
  public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);

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
    swerve.setDefaultCommand(new SwerveDriveDrive(driver));
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings() {
    // reset zero angle (intake away)
    driver.getDPadUp().onTrue(new SwerveDriveResetHeading());
    
    driver.getBottomButton()
      .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING));
  }

  private void configureOperatorBindings() {
    operator.getLeftTriggerButton()
      .whileTrue(new IntakeDeacquire())
      .whileTrue(new ConveyorSetMode(ConveyorMode.REVERSE));

    operator.getRightTriggerButton()
      .whileTrue(new IntakeAcquire())
      .onTrue(new IntakeExtend())
      .onFalse(new IntakeRetract());

    operator.getDPadUp().onTrue(new ShootHigh());
    operator.getDPadRight().onTrue(new ShootFar());
    operator.getDPadDown().onTrue(new ShootLow());
    operator.getDPadLeft()
        // .onTrue(new ArmDown())
        .onTrue(new ShootMid());
        // .onFalse(new ArmUp());

    operator.getTopButton()
        .onTrue(new ConveyorSetMode(ConveyorMode.INDEXING));
    operator.getRightButton()
        .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING));
    operator.getBottomButton()
        .onTrue(new ConveyorSetMode(ConveyorMode.STOP));
  }

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    autonChooser.addOption("Do Nothing", new DoNothingAuton());
    autonChooser.addOption("Myles", new MylesAuto(this, "myles"));
    
    autonChooser.setDefaultOption("MobilityAuton", new MobilityAuton(this, "Mobility"));
    autonChooser.addOption("OnePieceDock", new OnePieceDock(this, "OnePieceDock"));
    autonChooser.addOption("One Piece Mobility Dock", new OnePieceMobilityDock(this, "OnePieceMobilityDock"));
    autonChooser.addOption("OnePieceMobilityNonwire", new OnePieceMobilityNonwire(this, "OnePieceMobilityNonwire"));
    autonChooser.addOption("OnePieceMobilityWire", new OnePieceMobilityWire(this, "OnePieceMobilityWire"));

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
