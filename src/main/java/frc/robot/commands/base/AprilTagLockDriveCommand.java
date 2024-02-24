
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.Vision.VisionType;
import frc.robot.utils.Utility;

public class AprilTagLockDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;
  private Vision vision;
  private int targetTagId;

  public AprilTagLockDriveCommand(CommandSwerveDrivetrain drivetrain, Vision vision, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.controller = controller;
    targetTagId = 0;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // setPidControllers(vision.getTagId(VisionType.SHOOTER));
    vision.setPipeline(VisionType.SHOOTER, 1);
  }

  @Override
  public void execute() {
    boolean hasTarget = vision.hasTarget(VisionType.SHOOTER);
    boolean moveTo = controller.a().getAsBoolean();
    double tx = vision.getTx(VisionType.SHOOTER);
    int tagId = 0;
    if(hasTarget) {
      tagId = vision.getTagId(VisionType.SHOOTER);
      if(tagId == 3 || tagId == 8) {
        hasTarget = false;
      }
    }

    moveTo = moveTo && DriveConstants.moveToTags.contains(tagId);

    drivetrain.drive(
      getForward(vision.getTa(VisionType.SHOOTER), hasTarget, moveTo),
      getStrafe(tx, hasTarget),
      getRotation(tx, hasTarget),
      DriveMode.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }

  private double getForward(double ta, boolean hasTarget, boolean moveTo) {
    if(hasTarget && moveTo) {
      return drivetrain.getTargetLockForward(0.7 - ta, 0);
    }
    else {
      return Utility.getSpeed(controller.getLeftY()) * DriveConstants.MaxShootingSpeed;
    }
  }

  private double getStrafe(double tx, boolean hasTarget) {
    if(hasTarget) {
      return -drivetrain.getTargetLockStrafe(tx, 0);
    }
    else {
      return Utility.getSpeed(controller.getLeftX()) * DriveConstants.MaxShootingSpeed;
    }
  }

  private double getRotation(double tx, boolean hasTarget) {
    double poseRotation = drivetrain.getPose().getRotation().getDegrees();
    double poseError = poseRotation < 180 ? poseRotation + 180 : poseRotation - 180;
    return -drivetrain.getRotationLockRotation(poseError, 0);
  }

  // TODO: put cases for different tag ids
  private double getForwardSetpoint(int tagId) {
    return 0.0;
  }

  private double getRotationSetpoint(int tagId) {
    return 0.0;
  }
}
