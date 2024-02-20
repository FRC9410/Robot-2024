// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.utils.Utility;

public class DefaultDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  private double forward;
  private double strafe;
  private double rotation;

  public DefaultDriveCommand(CommandSwerveDrivetrain drivetrain, double forward, double strafe, double rotation) {
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = rotation;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    this.drivetrain.drive(
        Utility.getSpeed(forward) * DriveConstants.MaxSpeed,
        Utility.getSpeed(strafe) * DriveConstants.MaxSpeed,
        Utility.getSpeed(rotation) * DriveConstants.MaxSpeed,
        DriveMode.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }
}
