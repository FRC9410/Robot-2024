// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.group;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.base.AutoIntakeWristCommand;
import frc.robot.commands.base.AutoShootCommand;
import frc.robot.commands.base.AutoShooterWristCommand;
import frc.robot.commands.base.AutoVoltageFeedCommand;
import frc.robot.commands.base.IntakeCommand;
import frc.robot.commands.base.IntakeWristCommand;
import frc.robot.commands.base.ShootCommand;
import frc.robot.commands.base.ShooterWristCommand;
import frc.robot.commands.base.VoltageFeedCommand;
import frc.robot.subsystems.Subsystems;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class YeetNoteCommand extends SequentialCommandGroup {
  /** Creates a new YeetNoteCommand. */
  public YeetNoteCommand(Subsystems subsystems) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
     addCommands(
      new ParallelRaceGroup(
        new WaitCommand(1.0),
        new ShootCommand(subsystems.getShooter(), 105),
        new VoltageFeedCommand(subsystems.getShooter(), -50),
        new ShooterWristCommand(subsystems.getShooter(), 1.8, true),
        new IntakeWristCommand(subsystems.getIntake(),0.11)
      ),
      new ParallelCommandGroup(
        new ShootCommand(subsystems.getShooter(), 105),
        new VoltageFeedCommand(subsystems.getShooter(), -50),
        new ShooterWristCommand(subsystems.getShooter(), 1.8, false),
        new IntakeCommand(subsystems.getIntake(), 85, 8, 0),
        new IntakeWristCommand(subsystems.getIntake(),0.11)
      )
     ); 
  }
}
