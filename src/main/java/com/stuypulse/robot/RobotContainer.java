/************************ PROJECT SACROD ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.MobilityAuton;
import com.stuypulse.robot.commands.auton.OnePieceDock;
import com.stuypulse.robot.commands.auton.OnePieceMobilityDock;
import com.stuypulse.robot.commands.auton.OnePiecePickupMobilityNonwire;
import com.stuypulse.robot.commands.auton.OnePiecePickupMobilityWire;
import com.stuypulse.robot.commands.auton.ThreePieceDockWire;
import com.stuypulse.robot.commands.auton.ThreePieceMobilityNonwire;
import com.stuypulse.robot.commands.auton.ThreePieceMobilityWire;
import com.stuypulse.robot.commands.auton.TwoPieceDockWire;
import com.stuypulse.robot.commands.auton.TwoPieceMobilityNonwire;
import com.stuypulse.robot.commands.auton.TwoPieceMobilityWire;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.shooter.ShootHigh;
import com.stuypulse.robot.commands.swerve.SwerveDriveBalance;
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

import edu.wpi.first.math.geometry.Rotation2d;
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
    driver.getDPadUp()
      .onTrue(new SwerveDriveResetHeading());

    // reset zero angle (intake towards)
    driver.getDPadDown()
      .onTrue(new SwerveDriveResetHeading(Rotation2d.fromDegrees(180)));

    driver.getDPadLeft().whileTrue(new SwerveDriveBalance());
    
    // shoot
    // driver.getLeftTriggerButton()
    //   .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING).alongWith(new IntakeAcquire()));
    // driver.getRightTriggerButton()
    //   .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING).alongWith(new IntakeAcquire()));
    driver.getLeftTriggerButton()
      .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING));
    driver.getRightTriggerButton()
      .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING));
  }

  private void configureOperatorBindings() {
    // outtake
    operator.getLeftTriggerButton()
      .whileTrue(new IntakeDeacquire())
      .whileTrue(new ConveyorSetMode(ConveyorMode.REVERSE));

    // intake
    operator.getRightTriggerButton()
      .whileTrue(new IntakeAcquire())
      .onTrue(new IntakeExtend())
      .onFalse(new IntakeRetract());

    // shooter rpms
    operator.getDPadUp().onTrue(new ShootHigh());
    operator.getDPadDown().onTrue(new ShootLow());
    operator.getDPadLeft().onTrue(new ShootMid());
    operator.getDPadRight().onTrue(new ShootMid());

    // operator.getDPadRight().onTrue(new ShootFar());

    operator.getLeftBumper().onTrue(new ShooterStop());

    operator.getTopButton()
        .onTrue(new ConveyorSetMode(ConveyorMode.INDEXING));
    // shoot
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
    // autonChooser.addOption("Myles", new MylesAuto("myles"));
    
    autonChooser.setDefaultOption("Mobility", new MobilityAuton("Mobility"));
    autonChooser.addOption("1 Piece Dock", new OnePieceDock("OnePieceDock"));
    autonChooser.addOption("1.5 Piece Mobility Nonwire", new OnePiecePickupMobilityNonwire("OnePieceMobilityNonwire"));
    autonChooser.addOption("1 Piece Mobility Dock", new OnePieceMobilityDock("OnePieceDock", "OnePieceMobilityDockCS", "OnePieceMobilityUPCSDock"));
    autonChooser.addOption("1.5 Piece Wire", new OnePiecePickupMobilityWire("OnePieceMobilityWire"));
    autonChooser.addOption("2 Piece Dock Wire", new TwoPieceDockWire("OnePieceMobilityWire", "TwoPieceDockWire"));
    autonChooser.addOption("2 Piece Mobility Wire", new TwoPieceMobilityWire("OnePieceMobilityWire", "TwoPieceMobilityWireBack"));
    autonChooser.addOption("2 Piece Mobility Nonwire", new TwoPieceMobilityNonwire("OnePieceMobilityNonwire", "TwoPieceMobilityNonwireBack"));
    autonChooser.addOption("3 Piece Mobility Wire", new ThreePieceMobilityWire("OnePieceMobilityWire","ThreePieceMobilityWirePiece2","ThreePieceMobilityWirePiece3"));
    autonChooser.addOption("3 Piece Mobility Dock", new ThreePieceDockWire("OnePieceMobilityWire", "ThreePieceMobilityWirePiece2", "ThreePieceMobilityWirePiece3", "Insert path here go to CS"));
    autonChooser.addOption("3 Piece Mobility NonWire", new ThreePieceMobilityNonwire("OnePieceMobilityNonWire","insert path here","insert path here"));

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
