// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.Pose3d;
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
    SmartDashboard.putNumber("current Pose", robotContainer.getSubsystems().getDrivetrain().getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("x", robotContainer.getSubsystems().getDrivetrain().getPose().getX());
    SmartDashboard.putNumber("y", robotContainer.getSubsystems().getDrivetrain().getPose().getY());
    SmartDashboard.putNumber("shooter pipeline", robotContainer.getSubsystems().getVision().getPipeline(VisionType.SHOOTER));
    
    var alliance = DriverStation.getAlliance();
    SmartDashboard.putBoolean("has alliance", alliance.isPresent());
    SmartDashboard.putString("alliance color", alliance.get() == DriverStation.Alliance.Red ? "red" : "blue");
    
    boolean hasTarget = robotContainer.getSubsystems().getVision().hasTarget(VisionType.SHOOTER);
    double ta = robotContainer.getSubsystems().getVision().getTa(VisionType.SHOOTER);

    if(hasTarget && ta >= VisionConstants.kMaxShooterDistance
      && (robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 4
      || robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 7)) {
        robotContainer.getSubsystems().getLeds().setFadeAnimtation(255, 121, 198);
    }
    else {
      robotContainer.getSubsystems().getLeds().setFadeAnimtation(0, 255, 255);
    }
  } 

  @Override
  public void disabledInit() {
    robotContainer.setDisabledIdleMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    robotContainer.setEnabledIdleMode();
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
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-back");
    Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue("limelight-back");
    if (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagArea > 0.2) {
      Pose2d newPose = pose.toPose2d();
      newPose.rotateBy(Rotation2d.fromDegrees(180));
      robotContainer.getSubsystems().getDrivetrain().setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      robotContainer.getSubsystems().getDrivetrain().addVisionMeasurement(
        newPose,
        limelightMeasurement.timestampSeconds
      );
      robotContainer.getSubsystems().getDrivetrain().seedFieldRelative(newPose);
    } 
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
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-back");
    Pose3d pose = robotContainer.getAllianceColor() == "red"
      ? LimelightHelpers.getBotPose3d_wpiRed("limelight-back")
      : LimelightHelpers.getBotPose3d_wpiBlue("limelight-back");
    
    SmartDashboard.putNumber("target area", limelightMeasurement.avgTagArea);

    SmartDashboard.putNumber("tag count", limelightMeasurement.tagCount);
    if (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagArea > 0.1) {
      Pose2d newPose = pose.toPose2d();
      newPose.rotateBy(Rotation2d.fromDegrees(180));
      robotContainer.getSubsystems().getDrivetrain().setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
      robotContainer.getSubsystems().getDrivetrain().addVisionMeasurement(
        newPose,
        limelightMeasurement.timestampSeconds
      );
      robotContainer.getSubsystems().getDrivetrain().seedFieldRelative(newPose);
    } 

    boolean hasTarget = robotContainer.getSubsystems().getVision().hasTarget(VisionType.SHOOTER);
    double ta = robotContainer.getSubsystems().getVision().getTa(VisionType.SHOOTER);

    if (driverController.getRightTriggerAxis() > 0.5
      && hasTarget && ta >= VisionConstants.kMaxShooterDistance
      && (robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 4
      || robotContainer.getSubsystems().getVision().getTagId(VisionType.SHOOTER) == 7)) {
        robotContainer.getSubsystems().getShooter().setWristAngleSetpoint(ta);
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
