// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.base;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class AutoElevatorCommand extends Command {
  /** Creates a new IntakeIn. */
  private Elevator elevator;
  private double position;

  public AutoElevatorCommand(Elevator elevator, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      this.elevator.setElevatorPosition(this.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (this.speed < 0 && this.timer.hasElapsed(1) && Math.abs(this.intake.getVelocity()) < this.minCurrentDraw) {
    //   return true;
    // }
    return false;
  }
}
