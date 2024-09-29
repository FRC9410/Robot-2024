// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.base.AutoElevatorCommand;
import frc.robot.commands.base.ElevatorCommand;
import frc.robot.commands.base.FeedCommand;
import frc.robot.commands.base.IntakeCommand;
import frc.robot.commands.base.ShootCommand;
import frc.robot.commands.base.ShooterWristCommand;
import frc.robot.commands.base.VoltageFeedCommand;
import frc.robot.subsystems.Subsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTrapCommand extends SequentialCommandGroup {
  /** Creates a new StageFeederCommand. */
  public ScoreTrapCommand(Subsystems subsystems) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new WaitCommand(0.25),
        new FeedCommand(subsystems.getShooter(), -30, -8, 1.8),
        new IntakeCommand(subsystems.getIntake(), 30, 12, 0),
        new ShootCommand(subsystems.getShooter(), -40)
      ),
      new ParallelRaceGroup(
        new WaitCommand(1),
        new AutoElevatorCommand(subsystems.getElevator(), -80)
      ),
      new ParallelRaceGroup(
        new ShooterWristCommand(subsystems.getShooter(), 9, true),
        new WaitCommand(0.4)
      ),
      new ParallelRaceGroup(
        new WaitCommand(0.25),
        new ShooterWristCommand(subsystems.getShooter(), 9, false),
        new ShootCommand(subsystems.getShooter(), -10),
        new VoltageFeedCommand(subsystems.getShooter(), -10)
      ),
      new ParallelRaceGroup(
        new ShooterWristCommand(subsystems.getShooter(), 18, false),
        new WaitCommand(1)
      ),
      new ParallelRaceGroup(
        new ShooterWristCommand(subsystems.getShooter(), 10, false),
        new WaitCommand(.5)
      ),
      new ParallelRaceGroup(
        new ShooterWristCommand(subsystems.getShooter(), 18, false),
        new WaitCommand(.5)
      )
    );
  }

}
