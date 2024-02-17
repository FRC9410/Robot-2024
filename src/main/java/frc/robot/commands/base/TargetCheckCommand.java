// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Vision.VisionType;

public class TargetCheckCommand extends Command {
  private Subsystems subsystems;
  /** Creates a new TargetCheckCommand. */
  public TargetCheckCommand(Subsystems subsystems) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystems = subsystems;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(subsystems.getVision().getTx(VisionType.SHOOTER)) < 1
    && subsystems.getVision().getTy(VisionType.SHOOTER) >= VisionConstants.kMaxShooterDistance
    && subsystems.getVision().getTx(VisionType.SHOOTER) != 0
    && subsystems.getShooter().isShooterReady()){
      return true;
    }
    return false;
  }
}
