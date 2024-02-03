// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterWrist;
import frc.robot.Constants.RobotConstants;

public class Shooter extends SubsystemBase {
  private SparkPIDController pidController;
  private AbsoluteEncoder encoder;

  TalonFX feeder = new TalonFX(ShooterWrist.kFeederCanId, RobotConstants.kCtreCanBusName);

  TalonFX primaryWheel = new TalonFX(ShooterWrist.kPrimaryWheelCanId, RobotConstants.kCtreCanBusName);
  TalonFX secondaryWheel = new TalonFX(ShooterWrist.kSecondaryWheelCanId, RobotConstants.kCtreCanBusName);

  CANSparkMax primaryWrist = new CANSparkMax(ShooterWrist.kPrimaryWristCanId, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(ShooterWrist.kSecondaryWristCanId, MotorType.kBrushless);

  private double TuningP = ShooterWrist.kP;
  private double TuningD = ShooterWrist.kD;
  private double TuningFF = ShooterWrist.kFF;
  private double Tuningsetpoint = 0;
  private double TuningMaxOutput = ShooterWrist.kMaxOutput;
  private double TuningMinOutput = ShooterWrist.kMinOutput;

  /** Creates a new Shooter. */
  public Shooter() {
    this.primaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.setInverted(true);
    this.primaryWrist.setIdleMode(IdleMode.kBrake);
    this.secondaryWrist.setIdleMode(IdleMode.kBrake);


    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getAbsoluteEncoder(ShooterWrist.kAbsEncType);
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

  public void setShooterVelocity(double velocity) {
    this.primaryWheel.set(velocity);
    this.secondaryWheel.set(velocity * 0.8);
  }

  public void shooterOff() {
    this.primaryWheel.set(0);
    this.secondaryWheel.set(0);
  }

  public void setWristAngle(double angle) {
    this.pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void wristOff() {
    double currentPosition = encoder.getPosition();
    this.pidController.setReference(currentPosition, CANSparkMax.ControlType.kPosition);
  }

  public void feedOn(double speed) {
    this.feeder.set(speed);
  }

  public void feedOff() {
    this.feeder.set(0);
  }

  public SparkPIDController getPIDController() {
    return this.pidController;
  }

  public double getEncoderPosition() {
    return this.encoder.getPosition();
  }

}
