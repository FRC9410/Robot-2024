// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.base.IntakeCommand;
import frc.robot.commands.base.ShootCommand;
import frc.robot.commands.base.ShooterWristCommand;
import frc.robot.commands.base.VoltageIntakeCommand;
import frc.robot.commands.group.CenterNoteCommand;
import frc.robot.commands.group.CenterNoteCommand2;
import frc.robot.commands.group.IntakeNoteCommand;
import frc.robot.commands.group.ShootNoteCommand;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My joystick
  private Subsystems subsystems = new Subsystems();
  private CANdle candle = new CANdle(23, "rio");

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
   subsystems.getDrivetrain().setDefaultCommand( // Drivetrain will execute this command periodically
        subsystems.getDrivetrain().applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
           .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
           .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
       ));

    // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on start press
    driverController.start().onTrue(subsystems.getDrivetrain().runOnce(() -> subsystems.getDrivetrain().seedFieldRelative()));

    if (Utils.isSimulation()) {
      subsystems.getDrivetrain().seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    subsystems.getDrivetrain().registerTelemetry(logger::telemeterize);

   driverController.y().whileTrue(new IntakeNoteCommand(subsystems)); //.onFalse(new CenterNoteCommand(subsystems));
    driverController.rightBumper().whileTrue(new ShootNoteCommand(subsystems));
    // driverController.b().onTrue(new CenterNoteCommand2(subsystems));
    // driverController.a().whileTrue(new VoltageFeedCommand(subsystems.getShooter(), -90));
    driverController.x().whileTrue(new VoltageIntakeCommand(subsystems.getIntake(), -10, -6, 100));
    driverController.a().whileTrue(new ShooterWristCommand(subsystems.getShooter(), 0.5));
    driverController.b().whileTrue(new ShooterWristCommand(subsystems.getShooter(), -0.5));
    // driverController.y().whileTrue(new IntakeNoteCommand(subsystems));
  }

  public RobotContainer() {
    configureBindings();
    
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);
        // candle.animate(new FireAnimation(0.5, 0.7, 8, 0.7, 0.5));
        // candle.animate(new RainbowAnimation(1, 0.1, 8));
        // candle.animate(new RgbFadeAnimation(0.7, 0.4, 8));
        candle.animate(new SingleFadeAnimation(0, 255, 255, 0, 0.75, 8)); 
        // candle.animate(new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, 8));
  }
        

  public Subsystems getSubsystems() {
    return this.subsystems;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setEnabledIdleMode() {
    
    subsystems.getShooter().setDisableIdleMode();
    subsystems.getIntake().setDisableIdleMode();
  }

  public void setDisableIdleMode() {
    subsystems.getShooter().setEnableIdleMode();
    subsystems.getIntake().setEnableIdleMode();
  }
}
