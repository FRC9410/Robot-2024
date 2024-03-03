// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.Vision.VisionType;
import frc.robot.utils.Utility;

public class GamePieceLockedDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;
  private Vision vision;

  public GamePieceLockedDriveCommand(CommandSwerveDrivetrain drivetrain, Vision vision, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.controller = controller;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    boolean hasTarget = vision.hasTarget(VisionType.INTAKE);
    double tx = vision.getTx(VisionType.INTAKE);

    drivetrain.drive(
      getForward(vision.getTy(VisionType.INTAKE), vision.getTx(VisionType.INTAKE), hasTarget),
      getStrafe(tx, hasTarget),
      getRotation(tx, hasTarget),
      hasTarget ? DriveMode.ROBOT_RELATIVE : DriveMode.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }

  private double getForward(double ty, double tx, boolean hasTarget) {
    if(hasTarget && Math.abs(tx) < 8 && ty < 10) {
      return -0.4
       * DriveConstants.MaxSpeed;
    }
    else {
      return Utility.getSpeed(controller.getLeftY()) * DriveConstants.MaxShootingSpeed;
    }
  }

  private double getStrafe(double tx, boolean hasTarget) {
    return Utility.getSpeed(controller.getLeftX()) * DriveConstants.MaxShootingSpeed;
  }

  private double getRotation(double tx, boolean hasTarget) {
    if(hasTarget) {
      return -drivetrain.getTargetLockRotation(tx, 0);
    }
    else {
      return Utility.getSpeed(controller.getRightX()) * DriveConstants.MaxIntakingSpeed;
    }
  }
}
