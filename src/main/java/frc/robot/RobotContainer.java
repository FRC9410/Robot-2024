// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.base.DefaultDriveCommand;
import frc.robot.commands.base.GamePieceLockedDriveCommand;
import frc.robot.commands.base.AprilTagLockDriveCommand;
import frc.robot.commands.base.VoltageIntakeCommand;
import frc.robot.commands.group.AutoShootNoteCommand;
import frc.robot.commands.group.IntakeNoteCommand;
import frc.robot.commands.group.ScoreAmpCommand;
import frc.robot.commands.group.ShootNoteCommand;
import frc.robot.commands.group.ShootTrapCommand;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);
  private Subsystems subsystems = new Subsystems();
  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);
  public String allianceColor;
  // private Command runAuto = getAutoPath("Test1");

  public RobotContainer() {
    subsystems.getDrivetrain().registerTelemetry(logger::telemeterize);
    setAllianceColor();
    registerNamedCommands();
    configurePilotBindings();
    configureCopilotBindings();
    subsystems.getLeds().setFadeAnimtation(0, 255, 255);
  }

  private void configurePilotBindings() {
    driverController.start().onTrue(subsystems.getDrivetrain().runOnce(() -> {
      subsystems.getDrivetrain().resetPose();
      subsystems.getDrivetrain().seedFieldRelative();
    }));
    
    subsystems.getDrivetrain().setDefaultCommand(
      new DefaultDriveCommand(
        subsystems.getDrivetrain(),
        driverController));

    driverController.leftTrigger(0.5).whileTrue(
      new GamePieceLockedDriveCommand(
        subsystems.getDrivetrain(),
        subsystems.getVision(),
        driverController)
        .alongWith(new IntakeNoteCommand(subsystems)))
        .onFalse(new ParallelRaceGroup(
          new WaitCommand(1),
          new VoltageIntakeCommand(subsystems.getIntake(), -10, -6, 100)));

    driverController.rightTrigger(0.5).whileTrue(
      new AutoShootNoteCommand(subsystems)
        .alongWith(new AprilTagLockDriveCommand(
          subsystems.getDrivetrain(),
          subsystems.getVision(),
          driverController)));
    
    driverController.rightBumper().whileTrue(new ShootNoteCommand(subsystems));
  }

  private void configureCopilotBindings() {
    copilotController.x().whileTrue(new VoltageIntakeCommand(subsystems.getIntake(), -10, -6,100));
    copilotController.y().onTrue(new ScoreAmpCommand(subsystems));
    copilotController.b().whileTrue(new ShootTrapCommand(subsystems));
  }
        

  public Subsystems getSubsystems() {
    return this.subsystems;
  }

  // public Command getAutonomousCommand() {
  //   /* First put the drivetrain into auto run mode, then run the auto */
  //   return runAuto;
  // }

  public void setEnabledIdleMode() {
    
    subsystems.getShooter().setDisableIdleMode();
    subsystems.getIntake().setDisableIdleMode();
  }

  public void setDisableIdleMode() {
    subsystems.getShooter().setEnableIdleMode();
    subsystems.getIntake().setEnableIdleMode();
  }

  public CommandXboxController getDriverController() {
    return driverController;
  }

  // public Command getAutoPath(String pathName) {
  //     return new PathPlannerAuto(pathName);
  // }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("AutoShootNoteCommand", new AutoShootNoteCommand(subsystems));
    NamedCommands.registerCommand("IntakeNoteCommand", new IntakeNoteCommand(subsystems));
  }

  public void setAllianceColor() {
    DriverStation.getAlliance().ifPresent((alliance) ->
      allianceColor = (alliance == Alliance.Red ? "red" : "blue"));
  }
}
