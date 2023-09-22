/************************ PROJECT SACROD ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.AutoShoot;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.MylesAuto;
import com.stuypulse.robot.commands.conveyor.ConveyorSetMode;
import com.stuypulse.robot.commands.intake.*;
import com.stuypulse.robot.commands.shooter.ShooterSetRPM;
import com.stuypulse.robot.commands.shooter.ShooterStop;
import com.stuypulse.robot.commands.swerve.DrivetrainAlign;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveHome;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
// import com.stuypulse.robot.constants.Settings.Climber;
import com.stuypulse.robot.constants.Settings.Scoring;
import com.stuypulse.robot.subsystems.*;
import com.stuypulse.robot.subsystems.camera.Camera;
import com.stuypulse.robot.subsystems.shooter.*;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.robot.util.ConveyorMode;
import com.stuypulse.robot.subsystems.intake.*;
import com.stuypulse.robot.subsystems.climber.*;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // Subsystem
  public final Conveyor conveyor = new Conveyor();
  public final Shooter shooter = Shooter.getInstance();
  public final Intake intake = Intake.getInstance();
  public final Climber climber = Climber.getInstance();
  public final SwerveDrive swerve = SwerveDrive.getInstance();

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
    driver.getTopButton().onTrue(new SwerveDriveHome());
    driver.getBottomButton().whileTrue(new AutoShoot(this, driver));
    driver.getLeftButton().whileTrue(new DrivetrainAlign(camera, Scoring.PRIMARY_DISTANCE));

    operator.getLeftTriggerButton()
      .whileTrue(new IntakeDeacquire())
      .whileTrue(new ConveyorSetMode(ConveyorMode.REVERSE));

    operator.getRightTriggerButton()
      .whileTrue(new IntakeAcquire())
      .onTrue(new IntakeExtend())
      .onFalse(new IntakeRetract());

      //ask blay if shooing button should shoot torwards grid or does he want 
      //separate outtake and shooting button 
      //right now if facing grid and press button it shoots, if not facing grid
      //it outtakes 
    operator.getTopButton()
      .onTrue(new ConveyorSetMode(ConveyorMode.INDEXING));  
    operator.getRightButton()
      .whileTrue(new ConveyorSetMode(ConveyorMode.SHOOTING));
    operator.getBottomButton()
      .onTrue(new ConveyorSetMode(ConveyorMode.STOP));

    operator.getDPadUp().onTrue(new ShooterStop());
    operator.getDPadRight().onTrue(new ShooterSetRPM(Settings.Scoring.PRIMARY_RPM));
    operator.getDPadLeft().onTrue(new ShooterSetRPM(Settings.Scoring.SECONDARY_RPM));
    operator.getDPadDown().onTrue(new ShooterSetRPM(Settings.Scoring.TUNING_RPM));
  }

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
    autonChooser.addOption("Myles", new MylesAuto(this, "myles"));

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
