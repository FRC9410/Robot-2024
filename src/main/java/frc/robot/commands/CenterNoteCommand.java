// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Subsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterNoteCommand extends ParallelRaceGroup {
  /** Creates a new BalanceNoteCommand. */
  public CenterNoteCommand(Subsystems subsystems) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(5),
      new SequentialCommandGroup(
        new StageFeederCommand(subsystems, -40, -3, 2, 0.5),
        new WaitCommand(0.1),
        new EmptyFeederCommand(subsystems)
      )
    );
  }
}
