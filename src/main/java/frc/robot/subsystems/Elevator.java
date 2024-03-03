// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeWrist;
import frc.robot.Constants.ShooterWrist;

public class Elevator extends SubsystemBase {
  private SparkPIDController pidController;
  private RelativeEncoder encoder;
  private double setpoint;
  private double kP;
  private double kD;

  CANSparkMax primaryElevator = new CANSparkMax(ElevatorConstants.kPrimaryElevatorCanId, MotorType.kBrushless);
  CANSparkMax secondaryElevator = new CANSparkMax(ElevatorConstants.kSecondaryElevatorCanId, MotorType.kBrushless);


  /** Creates a new Elevator. */
  public Elevator() {
    this.primaryElevator.restoreFactoryDefaults();
    this.secondaryElevator.restoreFactoryDefaults();
    this.secondaryElevator.follow(primaryElevator, true);
    this.primaryElevator.setIdleMode(IdleMode.kBrake);
    this.secondaryElevator.setIdleMode(IdleMode.kBrake);

    this.pidController = primaryElevator.getPIDController();

    this.encoder = primaryElevator.getEncoder();
    this.encoder.setPosition(0);
    setpoint = 0;
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(ElevatorConstants.kP);
    this.pidController.setI(ElevatorConstants.kI);
    this.pidController.setD(ElevatorConstants.kD);
    this.pidController.setIZone(ElevatorConstants.kIz);
    this.pidController.setFF(ElevatorConstants.kFF);
    this.pidController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
    
    pidController.setSmartMotionMaxAccel(ElevatorConstants.maxAcc, 0);
    pidController.setSmartMotionMaxVelocity(ElevatorConstants.maxVel, 0);
    pidController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.allowedError, 0);
    
    SmartDashboard.putNumber("Elevator Position", setpoint); 
    SmartDashboard.putNumber("Elevator P", ElevatorConstants.kP); 
    SmartDashboard.putNumber("Elevator D", ElevatorConstants.kD); 
    kP = ElevatorConstants.kP;
    kD = ElevatorConstants.kD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Actual Position", this.encoder.getPosition()); 

    double newSetpoint = SmartDashboard.getNumber("Elevator Position", IntakeWrist.kMinRotation);
    if (setpoint != newSetpoint && newSetpoint < 0) {
      setpoint = newSetpoint;
      this.pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }

    double newKP = SmartDashboard.getNumber("Elevator P", IntakeWrist.kMinRotation);
    if (kP != newKP) {
      kP = newKP;
      this.pidController.setReference(kP, CANSparkMax.ControlType.kPosition);
    }
    
    double newKD = SmartDashboard.getNumber("Elevator D", IntakeWrist.kMinRotation);
    if (kD != newKD) {
      kD = newKD;
      this.pidController.setReference(kD, CANSparkMax.ControlType.kPosition);
    }
  }

  public void setElevatorPosition(double position) {
    this.pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void setElevatorPosition() {
    this.pidController.setReference(ElevatorConstants.kMaxRotation, CANSparkMax.ControlType.kPosition);
  }
 
  public void elevatorOff() {
    this.pidController.setReference(ElevatorConstants.kMinRotation, CANSparkMax.ControlType.kPosition);
  }

  public void setEnableIdleMode() {
    primaryElevator.setIdleMode(IdleMode.kBrake);
    secondaryElevator.setIdleMode(IdleMode.kBrake);
  }

  public void setDisabledIdleMode() {
    primaryElevator.setIdleMode(IdleMode.kCoast);
    secondaryElevator.setIdleMode(IdleMode.kCoast);
  }
}
