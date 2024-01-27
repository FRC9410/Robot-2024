// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeWrist;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CANSparkMax test = new CANSparkMax(56, MotorType.kBrushless);
  private CANSparkMax test2 = new CANSparkMax(10, MotorType.kBrushless);
  private SparkPIDController pidController;
  private RelativeEncoder encoder;
  private double kP= 0.5;
  private double kI= 0;
  private double kD= 0;
  private double setpoint = 0.75;
  private double enableTime = 0;
  private double maxVel = 2000;
  private double macAcc = 1500;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    this.test.restoreFactoryDefaults();
    this.pidController = test.getPIDController();
    this.encoder = test.getAlternateEncoder(ElevatorConstants.kAltEncType, ElevatorConstants.kCPR);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(kP);
    this.pidController.setI(kI);
    this.pidController.setD(kD);
    this.pidController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
    SmartDashboard.putNumber("setpoint", 0);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    pidController.setSmartMotionMaxAccel(IntakeWrist.maxAcc, 0);
    pidController.setSmartMotionMaxVelocity(IntakeWrist.maxVel, 0);
    pidController.setSmartMotionAllowedClosedLoopError(IntakeWrist.allowedError, 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("encoder value", encoder.getPosition());

    SmartDashboard.putNumber("intake output current", test2.getOutputCurrent());


    double newSetpoint = SmartDashboard.getNumber("setpoint", 0);
    if (setpoint != newSetpoint) {
      setpoint = newSetpoint;
      pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    double newkP = SmartDashboard.getNumber("kP", kP);
    if (kP != newkP) {
      kP = newkP;
      this.pidController.setP(kP);
    }

    double newkI = SmartDashboard.getNumber("kI", kI);
    if (kI != newkI) {
      kI = newkI;
      this.pidController.setI(kI);
    }

    double newkD = SmartDashboard.getNumber("kD", kD);
    if (kD != newkD) {
      kD = newkD;
      this.pidController.setD(kD);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

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
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
