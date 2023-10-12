/************************ PROJECT SACROD ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.AutoShoot;
import com.stuypulse.robot.commands.arm.ArmDown;
import com.stuypulse.robot.commands.arm.ArmUp;
import com.stuypulse.robot.commands.arm.ArmUp;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.MylesAuto;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.DrivetrainAlign;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveHome;
import com.stuypulse.robot.commands.swerve.SwerveDriveResetHeading;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
// import com.stuypulse.robot.constants.Settings.Climber;
import com.stuypulse.robot.constants.Settings.Scoring;
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
  public final Conveyor conveyor = Conveyor.getInstance();
  public final Shooter shooter = Shooter.getInstance();
  public final Intake intake = Intake.getInstance();
  public final Climber climber = Climber.getInstance();
  public final SwerveDrive swerve = SwerveDrive.getInstance();

  public final Camera camera = Camera.getInstance();

  // Gamepads
  public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

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
    driver.getDPadUp().onTrue(new SwerveDriveResetHeading());
    driver.getTopButton().onTrue(new SwerveDriveHome());
    // driver.getBottomButton().whileTrue(new AutoShoot(this, driver));
      
    driver.getBottomButton()
      .onTrue(new ConveyorSetMode(ConveyorMode.SHOOTING))
      .onFalse(new ConveyorSetMode(ConveyorMode.DEFAULT));

    // driver.getLeftButton().whileTrue(new DrivetrainAlign(camera,
    // Scoring.PRIMARY_DISTANCE));

    operator.getLeftTriggerButton()
        .whileTrue(new IntakeDeacquire())
        .whileTrue(new ConveyorSetMode(ConveyorMode.REVERSE));

    operator.getRightTriggerButton()
        .whileTrue(new IntakeAcquire())
        .onTrue(new IntakeExtend())
        .onFalse(new IntakeRetract());

    // ask blay if shooing button should shoot torwards grid or does he want
    // separate outtake and shooting button
    // right now if facing grid and press button it shoots, if not facing grid
    // it outtakes

    operator.getDPadUp().onTrue(new ShootHigh());
    operator.getDPadRight().onTrue(new ShootMid());
    operator.getDPadDown().onTrue(new ShootLow());
    // operator.getDPadLeft().onTrue(new ArmDown());
    operator.getDPadLeft()
        .onTrue(new ArmDown())
        .onTrue(new ShootCS())
        .onFalse(new ArmUp());

    operator.getTopButton()
        .onTrue(new ConveyorSetMode(ConveyorMode.INDEXING));
    operator.getRightButton()
        .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING));
    operator.getBottomButton()
        .onTrue(new ConveyorSetMode(ConveyorMode.STOP));

    // operator.getDPadLeft().onTrue(new
    // ShooterSetRPM(Settings.Scoring.TUNING_RPM));
  }

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
    autonChooser.addOption("Myles", new MylesAuto(this, "myles"));
    
    autonChooser.addOption("MobilityAuton", new MylesAuto(this, "Mobility"));
    autonChooser.addOption("OnePieceDock", new MylesAuto(this, "1 Piece + Dock"));
    autonChooser.addOption("OnePieceMobility", new MylesAuto(this, "1 Piece + mobility (nonwire)"));
    autonChooser.addOption("OnePieceMobilityWire", new MylesAuto(this, "1 piece + mobility (wire side)"));

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
