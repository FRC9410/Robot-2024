// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  /** Creates a new IntakeIn. */
  private Intake intake;
  private double speed;
  private double startTime;


  public IntakeCommand(Intake intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intake.intakeOn(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.intakeOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() > startTime +1 && intake.getRollerPowerDraw() > 15) {
      return true;
    }
    return false;
  }
}
