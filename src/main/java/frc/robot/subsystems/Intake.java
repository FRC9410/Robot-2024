// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeWrist;
import frc.robot.Constants.RobotConstants;

public class Intake extends SubsystemBase {

  private SparkPIDController pidController;
  private AbsoluteEncoder encoder;

  TalonFX intake = new TalonFX(IntakeWrist.kIntakeCanId, RobotConstants.kCtreCanBusName);
  CANSparkMax primaryWrist = new CANSparkMax(IntakeWrist.kPrimaryWristCanId, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(IntakeWrist.kSecondaryWristCanId, MotorType.kBrushless);

  private double TuningP = IntakeWrist.kP;
  private double TuningD = IntakeWrist.kD;
  private double TuningFF = IntakeWrist.kFF;
  private double Tuningsetpoint = 0;
  private double TuningMaxOutput = IntakeWrist.kMaxOutput;

  
  public Intake() {
    this.primaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.follow(primaryWrist, true);
    this.primaryWrist.setIdleMode(IdleMode.kBrake);
    this.secondaryWrist.setIdleMode(IdleMode.kBrake);

    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getAbsoluteEncoder(IntakeWrist.kAbsEncType);
    this.encoder.setZeroOffset(IntakeWrist.kOffset);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(IntakeWrist.kP);
    this.pidController.setI(IntakeWrist.kI);
    this.pidController.setD(IntakeWrist.kD);
    this.pidController.setOutputRange(IntakeWrist.kMinOutput, IntakeWrist.kMaxOutput);
    
    pidController.setSmartMotionMaxAccel(IntakeWrist.maxAcc, 0);
    pidController.setSmartMotionMaxVelocity(IntakeWrist.maxVel, 0);
    pidController.setSmartMotionAllowedClosedLoopError(IntakeWrist.allowedError, 0);

    this.pidController.setReference(0.05, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(double angle) {
      this.pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }
  

  public void wristOff() {
    this.pidController.setReference(IntakeWrist.kMinRotation, CANSparkMax.ControlType.kPosition);
  }

  public void intakeOn(double speed) {
    this.intake.set(speed);
  }


  public void intakeOff() {
    this.intake.set(0);
  }

  public SparkPIDController getPIDController() {
    return this.pidController;
  }

  public double getEncoderPosition() {
    return this.encoder.getPosition();
  }

  public double getRollerPowerDraw() {
    return this.intake.getSupplyCurrent().getValueAsDouble();
  }

}