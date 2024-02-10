// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public TalonFX intake = new TalonFX(IntakeWrist.kIntakeCanId, RobotConstants.kCtreCanBusName);
  CANSparkMax primaryWrist = new CANSparkMax(IntakeWrist.kPrimaryWristCanId, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(IntakeWrist.kSecondaryWristCanId, MotorType.kBrushless);

  private double TuningP = IntakeWrist.kP;
  private double TuningD = IntakeWrist.kD;
  private double TuningFF = IntakeWrist.kFF;
  private double Tuningsetpoint = 0;
  private double TuningMaxOutput = IntakeWrist.kMaxOutput;

  private static final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(86, 86, 0, 0, false, false, false);

  
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

    this.pidController.setReference(IntakeWrist.kMinRotation, CANSparkMax.ControlType.kPosition);

    setIntakeConfigs(intake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(intake.getSupplyCurrent());
  }

  public void setAngle(double angle) {
      this.pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }
  

  public void wristOff() {
    this.pidController.setReference(IntakeWrist.kMinRotation, CANSparkMax.ControlType.kPosition);
  }

  public void intakeOn(double velocity, double feedforward) {
    this.intake.setControl(torqueVelocity.withVelocity(velocity).withFeedForward(feedforward));
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

  private static void setIntakeConfigs(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.3; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0000; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    motor.getConfigurator().apply(configs);
  }

  public void setEnableIdleMode() {
    intake.setNeutralMode(NeutralModeValue.Brake);
    primaryWrist.setIdleMode(IdleMode.kBrake);
    secondaryWrist.setIdleMode(IdleMode.kBrake);
  }

  public void setDisableIdleMode() {
    intake.setNeutralMode(NeutralModeValue.Coast);
    primaryWrist.setIdleMode(IdleMode.kCoast);
    secondaryWrist.setIdleMode(IdleMode.kCoast);
  }
}