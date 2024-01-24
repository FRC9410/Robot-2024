// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swervedrive.DriveSubsystem;

public class DriveCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private double forward;
    private double strafe;
    private double rotation;
  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, double forward, double strafe, double rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        this.driveSubsystem = driveSubsystem;

        addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(
          -MathUtil.applyDeadband(this.forward, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(this.strafe, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(this.rotation, OIConstants.kDriveDeadband),
          true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true, true);
  }
}
