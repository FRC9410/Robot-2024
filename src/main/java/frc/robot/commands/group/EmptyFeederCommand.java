// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.base.FeedCommand;
import frc.robot.commands.base.IntakeCommand;
import frc.robot.subsystems.Subsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmptyFeederCommand extends ParallelRaceGroup {
  /** Creates a new EmptyFeederCommand. */
  public EmptyFeederCommand(Subsystems subsystems) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FeedCommand(subsystems.getShooter(), 40, 3, 0),
      new IntakeCommand(subsystems.getIntake(), -60, -6, 2)
    );
  }
}