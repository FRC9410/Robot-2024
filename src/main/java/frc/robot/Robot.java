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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private CANSparkMax intakeWristPrimary = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax intakeWristSecondary = new CANSparkMax(11, MotorType.kBrushless);
  private SparkPIDController pidController;
  private RelativeEncoder encoder;
  private double kP= 0.5;
  private double kI= 0;
  private double kD= 0;
  private double kF= 0;
  private double kOffset = 0;
  private double setpoint = 0.75;
  private double maxVel = 2000;
  private double macAcc = 1500;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // ShuffleboardTab tab = Shuffleboard.getTab("Test");
    // ShuffleboardLayout pidWidgetLayout = tab.getLayout("PIDController", BuiltInLayouts.kList);

    // pidWidgetLayout.withSize(2, 5); // Set the size of the layout
    // pidWidgetLayout.withPosition(0, 0); // Set the position of the layout

    // Add widgets to the layout
    // pidWidgetLayout.add("1 - encoder position", setpoint).withWidget(BuiltInWidgets.kDial).withPosition(0, 0);
    // pidWidgetLayout.add("2 - setpoint", setpoint);
    // pidWidgetLayout.add("3 - kP", kP);
    // pidWidgetLayout.add("4 - kD", kD);
    // pidWidgetLayout.add("5 - FF", 0);
    // pidWidgetLayout.add("6 - kOutputRange", 0);

    this.intakeWristPrimary.restoreFactoryDefaults();
    this.intakeWristSecondary.restoreFactoryDefaults();
    this.intakeWristPrimary.follow(intakeWristPrimary, true);
    this.pidController = intakeWristPrimary.getPIDController();
    this.encoder = intakeWristPrimary.getAlternateEncoder(ElevatorConstants.kAltEncType, ElevatorConstants.kCPR);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(kP);
    this.pidController.setI(kI);
    this.pidController.setD(kD);
    this.pidController.setOutputRange(IntakeWrist.kMinOutput, IntakeWrist.kMaxOutput);
    SmartDashboard.putNumber("setpoint", 0);
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kF", kF);

  
    

    pidController.setSmartMotionMaxAccel(IntakeWrist.maxAcc, 0);
    pidController.setSmartMotionMaxVelocity(IntakeWrist.maxVel, 0);
    pidController.setSmartMotionAllowedClosedLoopError(IntakeWrist.allowedError, 0);


  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("encoder value", encoder.getPosition());


    double newSetpoint = SmartDashboard.getNumber("setpoint", 0);
    if (setpoint != newSetpoint) {
      setpoint = newSetpoint;
      pidController.setReference(setpoint + kOffset, CANSparkMax.ControlType.kPosition);
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

    double newkF = SmartDashboard.getNumber("kF", kF);
    if (kF != newkF) {
      kF = newkF;
      this.pidController.setD(kF);
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
