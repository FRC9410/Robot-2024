// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain.DriveMode;
import frc.robot.utils.Utility;

public class DefaultDriveCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  CommandXboxController controller;
  public DefaultDriveCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.drive(
      Utility.getSpeed(controller.getLeftY()) * DriveConstants.MaxSpeed,
      Utility.getSpeed(controller.getLeftX()) * DriveConstants.MaxSpeed,
      Utility.getSpeed(controller.getRightX()) * DriveConstants.MaxSpeed,
      DriveMode.FIELD_RELATIVE);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, DriveMode.FIELD_RELATIVE);
  }
}
