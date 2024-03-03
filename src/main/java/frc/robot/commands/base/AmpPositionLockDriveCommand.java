
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.Vision.VisionType;
import frc.robot.utils.Utility;

public class AmpPositionLockDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  String allianceColor;

  public AmpPositionLockDriveCommand(CommandSwerveDrivetrain drivetrain, String allianceColor) {
    this.drivetrain = drivetrain;
    this.allianceColor = allianceColor;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d pose = drivetrain.getPose();
    double rotation = allianceColor == "red" ? 90.0 : 270.0;
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
