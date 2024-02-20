// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.subsystems.Vision.VisionType;
import frc.robot.utils.Utility;

public class SpeakerLockDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  private double forward;
  private double strafe;
  private Vision vision;

  public SpeakerLockDriveCommand(CommandSwerveDrivetrain drivetrain, Vision vision, double forward, double strafe) {
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.vision = vision;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    this.drivetrain.drive(
        Utility.getSpeed(forward) * DriveConstants.MaxShootingSpeed,
        Utility.getSpeed(strafe) * DriveConstants.MaxShootingSpeed,
        vision.getTx(VisionType.SHOOTER),
        DriveMode.TARGET_LOCK);
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }
}
