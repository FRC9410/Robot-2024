// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterWrist;

public class Shooter extends SubsystemBase {
private SparkPIDController pidController;
private RelativeEncoder encoder;

CANSparkMax feeder = new CANSparkMax(20, MotorType.kBrushless);

CANSparkMax primaryWheel = new CANSparkMax(21, MotorType.kBrushless);
CANSparkMax secondaryWheel = new CANSparkMax(22, MotorType.kBrushless);

CANSparkMax primaryWrist = new CANSparkMax(31, MotorType.kBrushless);
CANSparkMax secondaryWrist = new CANSparkMax(32, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    this.secondaryWheel.follow(primaryWheel);
    this.secondaryWheel.setInverted(true);

    this.secondaryWrist.follow(primaryWrist);
    this.secondaryWrist.setInverted(true);

    this.primaryWrist.restoreFactoryDefaults();

    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getAlternateEncoder(ShooterWrist.kAltEncType, ShooterWrist.kCPR);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(ShooterWrist.kP);
    this.pidController.setI(ShooterWrist.kI);
    this.pidController.setD(ShooterWrist.kD);
    this.pidController.setIZone(ShooterWrist.kIz);
    this.pidController.setFF(ShooterWrist.kFF);
    this.pidController.setOutputRange(ShooterWrist.kMinOutput, ShooterWrist.kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooterOn(double speed) {
    primaryWheel.set(speed);
  }

  public void shooterOff() {
    primaryWheel.set(0);
  }

  public void setWristAngle(double angle) {
    pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void wristOff() {
    double currentPosition = encoder.getPosition();
    pidController.setReference(currentPosition, CANSparkMax.ControlType.kPosition);
  }

  public void feedOn(double speed) {
    feeder.set(speed);
  }

  public void feedOff() {
    feeder.set(0);
  }
}
