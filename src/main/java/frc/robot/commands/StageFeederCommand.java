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
public class StageFeederCommand extends SequentialCommandGroup {
  /** Creates a new StageFeederCommand. */
  public StageFeederCommand(Subsystems subsystems, double speed, double minCurrentDraw, double time) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new FeedCommand(subsystems.getShooter(), speed, -4.5, 0),
      new WaitCommand(time)
      ),
      new ParallelRaceGroup(
        new FeedCommand(subsystems.getShooter(), -30, -8, 1.8),
        new IntakeCommand(subsystems.getIntake(), 30,12, 0)
      )
    );
  }

}
