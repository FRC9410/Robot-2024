// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
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
    this.secondaryWrist.follow(primaryWrist);
    this.secondaryWrist.setInverted(true);

    this.primaryWrist.restoreFactoryDefaults();

    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getAbsoluteEncoder(IntakeWrist.kAbsEncType);
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

  public void setPidValues(double newP, double newD, double newFF, double newSetpoint, double newOutputRange) {
    if (Tuningsetpoint != newSetpoint) {
      Tuningsetpoint = newSetpoint;
      pidController.setReference(Tuningsetpoint, CANSparkMax.ControlType.kPosition);
    }

    if (TuningP != newP) {
      TuningP = newP;
      this.pidController.setP(TuningP);
    }

    if (TuningD != newD) {
      TuningD = newD;
      this.pidController.setD(TuningD);
    }

    if (TuningFF != newFF) {
      TuningFF = newFF;
      this.pidController.setFF(TuningFF);
    }

    if (TuningMaxOutput != newOutputRange) {
      TuningMaxOutput = newOutputRange;
      this.pidController.setOutputRange(-newOutputRange, newOutputRange);
    }
  }

}