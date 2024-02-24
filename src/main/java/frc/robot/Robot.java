// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionType;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  CommandXboxController driverController ;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    driverController = robotContainer.getDriverController();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("ty", robotContainer.getSubsystems().getVision().getTy(VisionType.SHOOTER));
    SmartDashboard.putNumber("tx", robotContainer.getSubsystems().getVision().getTx(VisionType.SHOOTER));
    SmartDashboard.putNumber("ta", robotContainer.getSubsystems().getVision().getTa(VisionType.SHOOTER));
      var lastResult = LimelightHelpers.getLatestResults("limelight-back").targetingResults;

      Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

      if (lastResult.valid) {
        robotContainer.getSubsystems().getDrivetrain().addVisionMeasurement(llPose, Timer.getFPGATimestamp());
      }
  } 

  @Override
  public void disabledInit() {
    robotContainer.setEnabledIdleMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    robotContainer.setDisableIdleMode();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // robotContainer.getSubsystems().getMusic().playSong("jackSparrow");
  }

  @Override
  public void teleopPeriodic() {
      boolean hasTarget = robotContainer.getSubsystems().getVision().hasTarget(VisionType.SHOOTER);
      double ta = robotContainer.getSubsystems().getVision().getTa(VisionType.SHOOTER);
    if (driverController.getRightTriggerAxis() > 0.5
    && hasTarget && ta >= VisionConstants.kMaxShooterDistance
    && (robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 4 ||
    robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 7)) {
      robotContainer.getSubsystems().getShooter().setWristAngleSetpoint(ta);
    }
    
    if(hasTarget && ta >= VisionConstants.kMaxShooterDistance
    && (robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 4 ||
    robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 7)) {
      robotContainer.getSubsystems().getLeds().setFadeAnimtation(255, 121, 198);
    }
    else {
      robotContainer.getSubsystems().getLeds().setFadeAnimtation(0, 255, 255);
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void testExit() {}
  
}
