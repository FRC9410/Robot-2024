// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.base.VoltageIntakeCommand;
import frc.robot.commands.group.AutoShootNoteCommand;
import frc.robot.commands.group.IntakeNoteCommand;
import frc.robot.commands.group.ScoreAmpCommand;
import frc.robot.commands.group.ShootNoteCommand;
import frc.robot.commands.group.ShootTrapCommand;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Vision.VisionType;

public class RobotContainer {
  
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);
  private Subsystems subsystems = new Subsystems();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.MaxSpeed * OIConstants.LEFT_X_DEADBAND).withRotationalDeadband(DriveConstants.MaxAngularRate * OIConstants.RIGHT_X_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  private void configureBindings() {
   subsystems.getDrivetrain().setDefaultCommand( // Drivetrain will execute this command periodically
        subsystems.getDrivetrain().applyRequest(() -> drive.withVelocityX(-getSpeed(driverController.getLeftY()) * getMaxSpeed()) // Drive forward with negative Y (forward)
           .withVelocityY((-getSpeed(driverController.getLeftX()) * getMaxSpeed())) // Drive left with negative X (left)
           .withRotationalRate(getTurn()) // Drive counterclockwise with negative X (left)
       ));

    // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on start press
    driverController.start().onTrue(subsystems.getDrivetrain().runOnce(() -> subsystems.getDrivetrain().seedFieldRelative()));
    subsystems.getDrivetrain().registerTelemetry(logger::telemeterize);

    driverController.leftTrigger(0.5).whileTrue(new IntakeNoteCommand(subsystems)).onFalse(new ParallelRaceGroup(
      new WaitCommand(2),
      new VoltageIntakeCommand(subsystems.getIntake(), -10, -6, 100)
    ));
    driverController.rightBumper().whileTrue(new ShootNoteCommand(subsystems));
    driverController.rightTrigger(0.5).whileTrue(new AutoShootNoteCommand(subsystems));

    copilotController.x().whileTrue(new VoltageIntakeCommand(subsystems.getIntake(), -10, -6,100));
    copilotController.y().onTrue(new ScoreAmpCommand(subsystems));
    copilotController.b().whileTrue(new ShootTrapCommand(subsystems));
  }

  public RobotContainer() {
    configureBindings();
    subsystems.getLeds().setFadeAnimtation(0, 255, 255);
  }
        

  public Subsystems getSubsystems() {
    return this.subsystems;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

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

  private double getTurn() {
    double tx = subsystems.getVision().getTx(VisionType.SHOOTER);

    if(this.driverController.getRightTriggerAxis() > 0.5 && subsystems.getVision().hasTarget(VisionType.SHOOTER)) {
      double pGain = 0.08;

      return Math.abs(tx) < 0.5 ? 0 : -pGain * tx;
    }
    else{
      return -getSpeed(this.driverController.getRightX()) * getMaxSpeed();
    }
  }

  private double getMaxSpeed() {
    if(this.driverController.getRightTriggerAxis() > 0.5) {
      return DriveConstants.MaxShootingSpeed;
    }
    if(this.driverController.getLeftTriggerAxis() > 0.5) {
      return DriveConstants.MaxIntakingSpeed;
    }
    else{
      return DriveConstants.MaxSpeed;
    }
  }

  private double getSpeed(double speed) {
    double newSpeed = Math.pow(speed, 2);
    return speed > 0 ? newSpeed : -newSpeed;
    // return speed;
  }
}
