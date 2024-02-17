// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.base.VoltageIntakeCommand;
import frc.robot.commands.group.IntakeNoteCommand;
import frc.robot.commands.group.ScoreAmpCommand;
import frc.robot.commands.group.ShootNoteCommand;
import frc.robot.subsystems.Subsystems;

public class RobotContainer {
  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-back");
  
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);
  private Subsystems subsystems = new Subsystems();
  private CANdle candle = new CANdle(23, "rio");

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(DriveConstants.MaxSpeed * OIConstants.LEFT_X_DEADBAND).withRotationalDeadband(DriveConstants.MaxAngularRate * OIConstants.RIGHT_X_DEADBAND)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed);

  private void configureBindings() {
   subsystems.getDrivetrain().setDefaultCommand( // Drivetrain will execute this command periodically
        subsystems.getDrivetrain().applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * getMaxSpeed()) // Drive forward with negative Y (forward)
           .withVelocityY((-driverController.getLeftX() * getMaxSpeed())) // Drive left with negative X (left)
           .withRotationalRate(getTurn()) // Drive counterclockwise with negative X (left)
       ));

    // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driverController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    // reset the field-centric heading on start press
    driverController.start().onTrue(subsystems.getDrivetrain().runOnce(() -> subsystems.getDrivetrain().seedFieldRelative()));
    subsystems.getDrivetrain().registerTelemetry(logger::telemeterize);

    driverController.leftTrigger(0.5).whileTrue(new IntakeNoteCommand(subsystems)).onFalse(new ParallelRaceGroup(
      new WaitCommand(2),
      new VoltageIntakeCommand(subsystems.getIntake(), -10, -6, 100)
    ));
    driverController.rightBumper().whileTrue(new ShootNoteCommand(subsystems));
    driverController.rightTrigger(0.5).whileTrue(new ShootNoteCommand(subsystems));

    copilotController.x().whileTrue(new VoltageIntakeCommand(subsystems.getIntake(), -10, -6,100));
    copilotController.y().whileTrue(new ScoreAmpCommand(subsystems));
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

  public CommandXboxController getDriverController() {
    return driverController;
  }

  private double getTurn() {
    if(this.driverController.getRightTriggerAxis() > 0.5
    && Math.abs(table.getEntry("tx").getDouble(0.0)) > 0) {
      double pGain = 10.08;
      double tx = table.getEntry("tx").getDouble(0.0);

      return Math.abs(tx) < 1 ? 0 : pGain * tx;
    }
    else{
      return -this.driverController.getRightX() * getMaxSpeed();
    }
  }

  private double getMaxSpeed() {
    if(this.driverController.getRightTriggerAxis() > 0.5
    && Math.abs(table.getEntry("tx").getDouble(0.0)) > 0) {
      return DriveConstants.MaxShootingSpeed;
    }
    else{
      return DriveConstants.MaxSpeed;
    }
  }
}
