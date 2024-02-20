// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.Vision.VisionType;
import frc.robot.utils.Utility;

public class AprilTagLockDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  private double forward;
  private double strafe;
  private double rotation;
  private Vision vision;
  private boolean moveTo;
  private int targetTagId;

  public AprilTagLockDriveCommand(CommandSwerveDrivetrain drivetrain, Vision vision, double forward, double strafe, double rotation, boolean moveTo) {
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.vision = vision;
    this.rotation = rotation;
    this.moveTo = moveTo;
    targetTagId = 0;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    setPidControllers(vision.getTagId(VisionType.SHOOTER));
  }

  @Override
  public void execute() {
    boolean hasTarget = vision.hasTarget(VisionType.SHOOTER);
    double tx = vision.getTx(VisionType.SHOOTER);
    double rotationError = drivetrain.getPose().getRotation().getRadians();
    int tagId = 0;
    if(hasTarget) {
      tagId = vision.getTagId(VisionType.SHOOTER);
    }

    moveTo = moveTo && DriveConstants.moveToTags.contains(tagId);

    drivetrain.drive(
      getForward(vision.getTy(VisionType.SHOOTER), hasTarget, moveTo),
      getStrafe(tx, rotationError, hasTarget, moveTo),
      getRotation(tx, rotationError, hasTarget, moveTo),
      DriveMode.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }

  private double getForward(double ty, boolean hasTarget, boolean moveTo) {
    if(hasTarget && moveTo) {
      return drivetrain.getTargetLockForward(ty, 0);
    }
    else {
      return Utility.getSpeed(forward) * DriveConstants.MaxShootingSpeed;
    }
  }

  private double getStrafe(double tx, double rotationError, boolean hasTarget, boolean moveTo) {
    if(hasTarget && moveTo) {
      double strafeError = tx - Rotation2d.fromRadians(rotationError).getDegrees();
      return drivetrain.getTargetLockStrafe(strafeError, 0);
    }
    else {
      return Utility.getSpeed(strafe) * DriveConstants.MaxShootingSpeed;
    }
  }

  private double getRotation(double tx, double rotationError, boolean hasTarget, boolean moveTo) {
    if(hasTarget && moveTo) {
      return drivetrain.getTargetLockRotation(rotationError, 0);
    }
    else if(hasTarget) {
      return drivetrain.getTargetLockRotation(tx, 0);
    }
    else {
      return Utility.getSpeed(rotation) * DriveConstants.MaxIntakingSpeed;
    }
  }

  private void setPidControllers(int tagId) {
    if(tagId != targetTagId) {
      drivetrain.forwardPidController.setSetpoint(getForwardSetpoint(tagId));
      drivetrain.forwardPidController.reset();

      drivetrain.strafePidController.setSetpoint(0.0);
      drivetrain.strafePidController.reset();

      drivetrain.rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
      drivetrain.rotationPidController.setSetpoint(getRotationSetpoint(tagId));
      drivetrain.rotationPidController.reset();
      targetTagId = tagId;
    }
  }

  // TODO: put cases for different tag ids
  private double getForwardSetpoint(int tagId) {
    return 0.0;
  }

  private double getRotationSetpoint(int tagId) {
    return 0.0;
  }
}
