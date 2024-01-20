// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeWrist;

public class Intake extends SubsystemBase {

  private SparkPIDController pidController;
  private RelativeEncoder encoder;

  CANSparkMax intake = new CANSparkMax(10, MotorType.kBrushless);
  CANSparkMax primaryWrist = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(12, MotorType.kBrushless);

  
  public Intake() {
    this.secondaryWrist.follow(primaryWrist);
    this.secondaryWrist.setInverted(true);

    this.primaryWrist.restoreFactoryDefaults();

    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getAlternateEncoder(IntakeWrist.kAltEncType, IntakeWrist.kCPR);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(IntakeWrist.kP);
    this.pidController.setI(IntakeWrist.kI);
    this.pidController.setD(IntakeWrist.kD);
    this.pidController.setIZone(IntakeWrist.kIz);
    this.pidController.setFF(IntakeWrist.kFF);
    this.pidController.setOutputRange(IntakeWrist.kMinOutput, IntakeWrist.kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAngle(double angle) {
      pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }
  

  public void wristOff() {
    double currentPosition = encoder.getPosition();
    pidController.setReference(currentPosition, CANSparkMax.ControlType.kPosition);
  }

  public void intakeOn(double speed) {
    intake.set(speed);
  }


  public void intakeOff() {
    intake.set(0);
  }

}