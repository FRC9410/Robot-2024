// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.VisionType;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer robotContainer;
  private Tuning tuning;
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  CommandXboxController driverController ;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    driverController = robotContainer.getDriverController();
    // tuning = new Tuning(robotContainer.getSubsystems());
    // robotContainer.playSong();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("ty", robotContainer.getSubsystems().getVision().getTy(VisionType.SHOOTER));
    SmartDashboard.putNumber("tx", robotContainer.getSubsystems().getVision().getTx(VisionType.SHOOTER));
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
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // robotContainer.getSubsystems().getMusic().playSong("jackSparrow");
  }

  @Override
  public void teleopPeriodic() {
      double ty = robotContainer.getSubsystems().getVision().getTy(VisionType.SHOOTER);
    if (driverController.getRightTriggerAxis() > 0.5 && ty != 0 && ty >= VisionConstants.kMaxShooterDistance) {
      robotContainer.getSubsystems().getShooter().setShooterVelocitySetpoint(ty);
      robotContainer.getSubsystems().getShooter().setFeederVelocitySetpoint(ty);
      robotContainer.getSubsystems().getShooter().setWristAngleSetpoint(ty);
      robotContainer.getSubsystems().getIntake().setWristAngleSetpoint(ty);
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
    // tuning.updateTuning(robotContainer.getSubsystems());

  }

  @Override
  public void testExit() {}
  
}
