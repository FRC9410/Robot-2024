
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;

public class StageLockDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  boolean isStageLeft;
  String xCoordinate;
  String yCoordinate;
  double rotation;

  public StageLockDriveCommand(CommandSwerveDrivetrain drivetrain, boolean isStageLeft) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
      xCoordinate = "2.2";
    if (isStageLeft) {
      yCoordinate = "5.5";
      rotation = 240.0;
    } else {
      yCoordinate = "6.5";
      rotation = 120.0;
    }
  }

  @Override
  public void execute() {
    Pose2d pose = drivetrain.getPose();
    drivetrain.drive(
      getForward(pose.getX()),
      getStrafe(pose.getY()),
      getRotation(rotation),
      DriveMode.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }

  private double getForward(double error) {
    return -(1.75 - error) * 3;
  }

  private double getStrafe(double error) {
    return -(0.65 - error) * 3;
  }

  private double getRotation(double setpoint) {
    double currentPoseRotation = drivetrain.getPose().getRotation().getDegrees();
    double distance = setpoint - currentPoseRotation;
    double error = distance < 180 && distance > -180 ? distance : distance > 180 ? distance - 360 : distance + 360;

    return drivetrain.getRotationLockRotation(error, 0);
  }
}
