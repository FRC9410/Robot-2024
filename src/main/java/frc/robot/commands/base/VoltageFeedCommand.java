// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class VoltageFeedCommand extends Command {
  /** Creates a new FeedIn. */
  private Shooter shooter;
  private double speed;

  public VoltageFeedCommand(Shooter shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.setFeederVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this.shooter.setFeederVelocity(0);
    this.shooter.feedOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
