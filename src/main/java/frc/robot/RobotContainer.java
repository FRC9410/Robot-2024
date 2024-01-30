// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeWristCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Subsystems;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The driver's controller
  private XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  private Subsystems subsystems = new Subsystems();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, Button.kA.value).toggleOnTrue(new ShootCommand(subsystems.getShooter(), 1));
    new JoystickButton(driverController, Button.kX.value).toggleOnTrue(new FeedCommand(subsystems.getShooter(), 1));
    new JoystickButton(driverController, Button.kY.value).toggleOnTrue(new IntakeCommand(subsystems.getIntake(), 1));
    new JoystickButton(driverController, Button.kB.value).toggleOnTrue(new IntakeCommand(subsystems.getIntake(), -1));
    new JoystickButton(driverController, Button.kLeftBumper.value).onTrue(new IntakeWristCommand(subsystems.getIntake(), 0.2));
    new JoystickButton(driverController, Button.kRightBumper.value).onTrue(new IntakeWristCommand(subsystems.getIntake(), -0.2));
  }

  public Subsystems getSubsystems() {
    return this.subsystems;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     // Create config for trajectory
//   }
}