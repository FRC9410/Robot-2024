// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterWrist;
import frc.robot.commands.group.IntakeNoteCommand;
import frc.robot.Constants.IntakeWrist;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utils.LinearInterpolator;

public class Shooter extends SubsystemBase {
  private SparkPIDController pidController;
  private RelativeEncoder encoder;

  private LinearInterpolator wristAngleInterpolator;

  private static final VelocityVoltage secondaryWheelVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private static final VelocityVoltage primaryWheelVoltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private static final VelocityVoltage feederVoltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
  private static final VelocityVoltage voltageVelocityFoc = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private static final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(86, 86, 0, 0, false, false, false);
  private static final NeutralOut brake = new NeutralOut();

  private TalonFX feeder = new TalonFX(ShooterWrist.kFeederCanId, RobotConstants.kCtreCanBusName);

  private TalonFX primaryWheel = new TalonFX(ShooterWrist.kPrimaryWheelCanId, RobotConstants.kCtreCanBusName);
  private TalonFX secondaryWheel = new TalonFX(ShooterWrist.kSecondaryWheelCanId, RobotConstants.kCtreCanBusName);

  CANSparkMax primaryWrist = new CANSparkMax(ShooterWrist.kPrimaryWristCanId, MotorType.kBrushless);
  CANSparkMax secondaryWrist = new CANSparkMax(ShooterWrist.kSecondaryWristCanId, MotorType.kBrushless);

  private double TuningP = ShooterWrist.kP;
  private double TuningD = ShooterWrist.kD;
  private double TuningFF = ShooterWrist.kFF;
  private double Tuningsetpoint = 0;
  private double TuningMaxOutput = ShooterWrist.kMaxOutput;
  private double TuningMinOutput = ShooterWrist.kMinOutput;


  private double setpoint;
  
  private double wristSetpoint;

  /** Creates a new Shooter. */
  public Shooter() {
    this.wristAngleInterpolator = new LinearInterpolator(ShooterWrist.wristAngles);
    this.primaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.restoreFactoryDefaults();
    this.secondaryWrist.follow(primaryWrist, true);
    this.primaryWrist.setIdleMode(IdleMode.kBrake);
    this.secondaryWrist.setIdleMode(IdleMode.kBrake);


    this.pidController = primaryWrist.getPIDController();

    this.encoder = primaryWrist.getEncoder();
    this.encoder.setPosition(-0.2);
    this.pidController.setFeedbackDevice(encoder);

    this.pidController.setP(ShooterWrist.kP);
    this.pidController.setI(ShooterWrist.kI);
    this.pidController.setD(ShooterWrist.kD);
    this.pidController.setIZone(ShooterWrist.kIz);
    this.pidController.setFF(ShooterWrist.kFF);
    this.pidController.setOutputRange(ShooterWrist.kMinOutput, ShooterWrist.kMaxOutput);
    
    pidController.setSmartMotionMaxAccel(ShooterWrist.maxAcc, 0);
    pidController.setSmartMotionMaxVelocity(ShooterWrist.maxVel, 0);
    pidController.setSmartMotionAllowedClosedLoopError(ShooterWrist.allowedError, 0);
    
    this.setpoint = ShooterWrist.kMinRotation;
    this.pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    // This method will be called once per scheduler run


    setShooterConfigs(primaryWheel);
    setShooterConfigs(secondaryWheel);
    setFeederConfigs(feeder);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(this.encoder.getPosition());
    
    // double newSetpoint = SmartDashboard.getNumber("shooter setpoint", IntakeWrist.kMinRotation);
    // if (setpoint != newSetpoint) {
    //   setpoint = newSetpoint;
    //   this.pidController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    // }

    SmartDashboard.putNumber("Shooter Wrist Actual", this.encoder.getPosition());
    SmartDashboard.putNumber("Shooter Wrist Setpoint", wristSetpoint);
    SmartDashboard.putNumber("Primary Wheel Actual", primaryWheel.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Secondary Wheel Actual", secondaryWheel.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Primary Wheel Current", primaryWheel.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Primary Wheel Voltage", primaryWheel.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Secondary Wheel Current", secondaryWheel.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Secondary Wheel Voltage", secondaryWheel.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Secondary Wheel Setpoint", primaryWheelVoltageVelocity.Velocity);
    SmartDashboard.putNumber("Primary Wheel Setpoint", secondaryWheelVoltageVelocity.Velocity);
    // System.out.println(primaryWheel.getVelocity());
  }

  public void setShooterVelocity(double velocity) {
    this.primaryWheel.setControl(primaryWheelVoltageVelocity.withVelocity(-velocity).withFeedForward(-ShooterConstants.kFF)); //-100
    this.secondaryWheel.setControl(secondaryWheelVoltageVelocity.withVelocity(velocity-5).withFeedForward(ShooterConstants.kFF)); //95
    // primaryWheel.set(-100);
    // secondaryWheel.set(100);
  }

  public void setShooterVelocity() {
    this.primaryWheel.setControl(primaryWheelVoltageVelocity.withVelocity(-90/*-ShooterConstants.kSpeakerShooterSpeed*/).withFeedForward(-ShooterConstants.kFF)); //-100
    this.secondaryWheel.setControl(secondaryWheelVoltageVelocity.withVelocity(85/*ShooterConstants.kSpeakerShooterSpeed -5*/).withFeedForward(ShooterConstants.kFF)); //95
    
    // primaryWheel.set(-100);
    // secondaryWheel.set(100);
  }

  public void shooterOff() {
    this.primaryWheel.setControl(brake);
    this.secondaryWheel.setControl(brake);
  }

  public void setWristAngle(double angle) {
    this.pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public void setWristAngle() {
    this.pidController.setReference(wristSetpoint, CANSparkMax.ControlType.kPosition);
  }

  public void wristOff() {
    double currentPosition = encoder.getPosition();
    this.pidController.setReference(currentPosition, CANSparkMax.ControlType.kPosition);
  }

  public void feedOn(double velocity, double feedforward) {
    this.feeder.setControl(feederVoltageVelocity.withVelocity(velocity));
  }

  public void feedOff() {
    this.feeder.setControl(brake);
  }

  public void setFeederVelocity(double velocity) {
    this.feeder.setControl(feederVoltageVelocity.withVelocity(velocity));

  }

  public void setFeederVelocity() {
    this.feeder.setControl(voltageVelocityFoc.withVelocity(ShooterConstants.kSpeakerFeederSpeed));
  }

  public SparkPIDController getPIDController() {
    return this.pidController;
  }

  public double getEncoderPosition() {
    return this.encoder.getPosition();
  }

  public double getFeederPowerDraw() {
    return this.feeder.getSupplyCurrent().getValueAsDouble();
  }

  public TalonFX getFeederMotor() {
    return this.feeder;
  }

  public TalonFX getPrimaryWheelMotor() {
    return this.primaryWheel;
  }

  public TalonFX getSecondaryWheelMotor() {
    return this.secondaryWheel;
  }

  private static void setShooterConfigs(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 16;
    configs.Voltage.PeakReverseVoltage = -16;

    motor.getConfigurator().apply(configs);
  }

  private static void setFeederConfigs(TalonFX motor) {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.13; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0000; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    motor.getConfigurator().apply(configs);
  }

  public void setWristAngleSetpoint(double ty){
    double newWristSetpoint = wristAngleInterpolator.getInterpolatedValue(ty) >= ShooterWrist.kMinRotation
    && wristAngleInterpolator.getInterpolatedValue(ty) <= ShooterWrist.kMaxRotation ?
    wristAngleInterpolator.getInterpolatedValue(ty) : 0.0;

    if(Math.abs(newWristSetpoint - setpoint) > 0.15 && newWristSetpoint != 0.0) {
      wristSetpoint = newWristSetpoint;
    }
  }

  public boolean isShooterReady() {
    return Math.abs(primaryWheel.getVelocity().getValueAsDouble()) > 70
    && Math.abs(secondaryWheel.getVelocity().getValueAsDouble()) > 70
    && Math.abs(feeder.getVelocity().getValueAsDouble()) > Math.abs(ShooterConstants.kSpeakerFeederSpeed) - 5
    && encoder.getPosition() > (wristSetpoint - 0.2)
    && encoder.getPosition() < (wristSetpoint);
  }
  public void setEnableIdleMode() {
    feeder.setNeutralMode(NeutralModeValue.Coast);
    primaryWheel.setNeutralMode(NeutralModeValue.Coast);

    secondaryWheel.setNeutralMode(NeutralModeValue.Coast);
    primaryWrist.setIdleMode(IdleMode.kBrake);
    secondaryWrist.setIdleMode(IdleMode.kBrake);

  }

  public void setDisableIdleMode() {
    feeder.setNeutralMode(NeutralModeValue.Coast);
    primaryWheel.setNeutralMode(NeutralModeValue.Coast);
    secondaryWheel.setNeutralMode(NeutralModeValue.Coast);
    primaryWrist.setIdleMode(IdleMode.kCoast);
    secondaryWrist.setIdleMode(IdleMode.kCoast);
  }
}
